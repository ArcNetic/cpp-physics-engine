#pragma once

namespace Constants
{
    // Gravity Acceleration (pixels/s^2)
    constexpr float GRAVITY = 980.f;

    // Default Restitution (used as fallback; bodies can override per-instance)
    constexpr float DEFAULT_RESTITUTION = 0.7f;

    // Window dimensions
    constexpr float WINDOW_WIDTH = 800.f;
    constexpr float WINDOW_HEIGHT = 600.f;

    // Friction coefficients (Coulomb model)
    constexpr float FRICTION_STATIC = 0.4f;
    constexpr float FRICTION_DYNAMIC = 0.2f;
}