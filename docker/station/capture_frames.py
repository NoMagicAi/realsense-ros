#!/usr/bin/env python3
# Copyright(c) NoMagic. All Rights Reserved.
#
# Station test client for the NoMagic `get_latest_frame` service. For each requested
# stream it repeatedly calls <ns>/<stream>/get_latest_frame, measures acquisition
# timing (round-trip plus the server-side duration fields carried in the response),
# saves each returned image to the output directory, and writes a CSV + prints a
# summary. Auto-discovers the services so it is namespace-agnostic.
import argparse
import csv
import os
import statistics
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from realsense2_camera_msgs.srv import GetLatestFrame

import cv2


GLF_TYPE = 'realsense2_camera_msgs/srv/GetLatestFrame'


def discover_services(node):
    """Return {stream_key: full_service_name} for every GetLatestFrame service."""
    found = {}
    for name, types in node.get_service_names_and_types():
        if name.endswith('/get_latest_frame') and GLF_TYPE in types:
            key = name[:-len('/get_latest_frame')].rsplit('/', 1)[-1]
            found[key] = name
    return found


def save_image(bridge, img, path, node):
    """Save a sensor_msgs/Image; 8-bit as PNG, non-8-bit (e.g. Z16 depth) as PNG + .npy."""
    try:
        cv = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
        enc = img.encoding.lower()
        to_write = cv
        if enc == 'rgb8':
            to_write = cv2.cvtColor(cv, cv2.COLOR_RGB2BGR)
        elif enc == 'rgba8':
            to_write = cv2.cvtColor(cv, cv2.COLOR_RGBA2BGRA)
        cv2.imwrite(path, to_write)
        if cv.dtype != np.uint8:
            np.save(os.path.splitext(path)[0] + '.npy', cv)
        return True
    except Exception as e:  # noqa: BLE001 - best-effort save, keep measuring
        node.get_logger().warn(f'save failed for {path}: {e}')
        return False


def main():
    ap = argparse.ArgumentParser(description='Capture + time get_latest_frame calls.')
    ap.add_argument('--streams', nargs='*', default=['color', 'aligned_depth_to_color'],
                    help='stream keys to capture (e.g. color depth aligned_depth_to_color infra1)')
    ap.add_argument('--num', type=int, default=10, help='shots per stream')
    ap.add_argument('--interval', type=float, default=0.0, help='seconds between shots')
    ap.add_argument('--output', default='/data', help='output directory (mounted volume)')
    ap.add_argument('--timeout', type=float, default=60.0, help='seconds to wait for services / each call')
    args = ap.parse_args()

    os.makedirs(args.output, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('nomagic_get_latest_frame_capture')
    bridge = CvBridge()

    # Wait for the requested services to appear (driver may still be starting).
    deadline = time.time() + args.timeout
    services = {}
    while time.time() < deadline:
        avail = discover_services(node)
        services = {s: avail[s] for s in args.streams if s in avail}
        if len(services) == len(args.streams):
            break
        rclpy.spin_once(node, timeout_sec=0.5)

    if not services:
        node.get_logger().error(f'No matching get_latest_frame services for {args.streams}.')
        node.get_logger().error(f'Discovered: {sorted(discover_services(node))}')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    missing = [s for s in args.streams if s not in services]
    if missing:
        node.get_logger().warn(f'Streams not found (skipping): {missing}')

    clients = {}
    for key, sname in services.items():
        cli = node.create_client(GetLatestFrame, sname)
        if not cli.wait_for_service(timeout_sec=args.timeout):
            node.get_logger().error(f'service {sname} did not become ready')
            continue
        clients[key] = cli
        node.get_logger().info(f'Using service {sname}')

    rows = []
    roundtrip = {k: [] for k in clients}
    server_side = {k: [] for k in clients}

    for i in range(args.num):
        for key, cli in clients.items():
            req = GetLatestFrame.Request()
            t0 = time.perf_counter()
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
            t1 = time.perf_counter()
            resp = future.result()
            if resp is None:
                node.get_logger().error(f'call failed: {key} shot {i}')
                continue
            if not resp.success:
                node.get_logger().error(
                    f'server-side failure: {key} shot {i}: {resp.error_message}')
                continue
            roundtrip_ms = (t1 - t0) * 1000.0
            server_ms = (resp.wait_for_frames_duration + resp.reset_temporal_filter_duration
                         + resp.filtering_duration + resp.depth_alignment_duration) * 1000.0
            roundtrip[key].append(roundtrip_ms)
            server_side[key].append(server_ms)

            fname = f'{key}_{i:03d}.png'
            saved = save_image(bridge, resp.image, os.path.join(args.output, fname), node)
            rows.append({
                'shot': i, 'stream': key, 'file': fname if saved else '',
                'roundtrip_ms': round(roundtrip_ms, 3),
                'server_total_ms': round(server_ms, 3),
                'wait_for_frames_ms': round(resp.wait_for_frames_duration * 1000, 3),
                'reset_temporal_filter_ms': round(resp.reset_temporal_filter_duration * 1000, 3),
                'filtering_ms': round(resp.filtering_duration * 1000, 3),
                'depth_alignment_ms': round(resp.depth_alignment_duration * 1000, 3),
                'frame_timestamp': resp.frame_timestamp,
                'request_timestamp': resp.request_timestamp,
                'response_timestamp': resp.response_timestamp,
                'width': resp.image.width, 'height': resp.image.height,
                'encoding': resp.image.encoding,
            })
            node.get_logger().info(
                f'[{key} #{i}] roundtrip={roundtrip_ms:6.1f}ms  server={server_ms:6.1f}ms  '
                f'(wait={resp.wait_for_frames_duration*1000:.1f} '
                f'filt={resp.filtering_duration*1000:.1f} '
                f'align={resp.depth_alignment_duration*1000:.1f})  '
                f'{resp.image.width}x{resp.image.height} {resp.image.encoding}')
        if args.interval > 0:
            time.sleep(args.interval)

    csv_path = os.path.join(args.output, 'acquisition_timing.csv')
    if rows:
        with open(csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    print('\n===== Acquisition timing summary (ms) =====')
    print(f'{"stream":<28}{"n":>4}{"rt_mean":>9}{"rt_min":>8}{"rt_max":>8}{"rt_p50":>8}{"srv_mean":>10}')
    for key in clients:
        rt = roundtrip[key]
        sv = server_side[key]
        if rt:
            print(f'{key:<28}{len(rt):>4}{statistics.mean(rt):>9.1f}{min(rt):>8.1f}'
                  f'{max(rt):>8.1f}{statistics.median(rt):>8.1f}{statistics.mean(sv):>10.1f}')
    print(f'\nSaved {len(rows)} image(s) + {csv_path}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
