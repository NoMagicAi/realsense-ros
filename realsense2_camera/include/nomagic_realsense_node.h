// Copyright(c) NoMagic. All Rights Reserved.
//
// Helper types for the NoMagic additions; the members/methods themselves are
// declared in the NOMAGIC block of base_realsense_node.h.

#pragma once

#include <chrono>

namespace realsense2_camera
{
    // Simple steady-clock stopwatch used to fill the timing fields of the
    // GetLatestFrame response (wait/reset/filtering/alignment durations).
    struct Clock
    {
        using InternalClock = std::chrono::steady_clock;

        Clock() : start(InternalClock::now()) {}
        void restart() { start = InternalClock::now(); }

        double getElapsedSecs()
        {
            return static_cast<std::chrono::duration<double>>(InternalClock::now() - start).count();
        }

    private:
        InternalClock::time_point start;
    };
}
