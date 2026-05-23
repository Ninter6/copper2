#pragma once

#include <chrono>
#include <iostream>

struct Timer {
    using clock = std::chrono::high_resolution_clock;
    using duration = std::chrono::high_resolution_clock::duration;
    using duration_ms = std::chrono::duration<double, std::milli>;
    using time_point = std::chrono::high_resolution_clock::time_point;

    time_point start = clock::now();
    time_point last = clock::now();

    [[nodiscard]] duration elapsed() const {
        return clock::now() - start;
    }
    [[nodiscard]] duration_ms elapsed_ms() const {
        return std::chrono::duration_cast<duration_ms>(elapsed());
    }

    void tick() {
        last = clock::now();
    }

    [[nodiscard]] duration tock() const {
        return clock::now() - last;
    }
    [[nodiscard]] duration_ms tock_ms() const {
        return std::chrono::duration_cast<duration_ms>(tock());
    }
};

inline Timer g_timer;

#define TICK() do g_timer.tick(); while (0)
#define TOCK() \
    do {std::cout << "TOCK():" << __LINE__ << ": " << g_timer.tock_ms() << "\n"; \
    } while (0);
