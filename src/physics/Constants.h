#pragma once

namespace Constants
{
    // Floor
    constexpr float FLOOR_Y = 550.f;

    // Gravity Acceleration (pixels/s^2)
    constexpr float GRAVITY = 980.f;

    // Default Restitution (used as fallback; bodies can override per-instance)
    constexpr float DEFAULT_RESTITUTION = 0.7f;

    // Sleep Threshold — minimum velocity magnitude to consider a body "at rest"
    constexpr float SLEEP_THRESHOLD = 30.f;

    // Window dimensions
    constexpr float WINDOW_WIDTH = 800.f;
    constexpr float WINDOW_HEIGHT = 600.f;
}