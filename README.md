# C++ 2D Physics Engine

A robust, custom-built 2D physics engine written entirely from scratch in C++. The engine's core physics math is **100% decoupled** from any external frameworks, allowing it to be easily ported into any custom engine or renderer.

Currently, the project uses [SFML 3](https://www.sfml-dev.org/) solely for window creation, input handling, and rendering the sandbox environment.

## 🚀 Key Features

### Core Physics Architecture
* **RigidBody Dynamics:** Simulates mass, linear velocity, forces, and gravity.
* **Rotational Dynamics:** Full support for angular velocity, torque, and exact moments of inertia for different shape primitives.
* **Framework Agnostic Math:** The entire physics core uses a custom `Physics::Vector2` implementation, meaning `src/physics` has zero dependencies on SFML or external libraries.

### Collision System
* **Separating Axis Theorem (SAT):** Exact polygon overlap detection for rotated bounding boxes and circles.
* **Contact Point Generation:** Accurately finds the deepest penetrating vertices on incident faces to apply torque at the correct lever-arm locations.
* **Spatial Partitioning (Broad-Phase):** Implements a highly efficient `SpatialGrid` (uniform grid) that eliminates O(N^2) bottlenecks by grouping nearby objects into cells before testing for collisions.

### Collision Resolution
* **Impulse-Based Resolution:** Uses exact impulse calculations for normal forces and restitution (bounciness).
* **Friction & Coulomb Clamping:** Calculates static and dynamic tangential friction impulses.
* **Baumgarte Stabilization:** Corrects floating-point positional drift, preventing objects from slowly sinking into one another.

### Constraints & Joints
* **Distance Joints:** Capable of tethering bodies together to simulate pendulums, springs, or chains.

## 🛠️ Tech Stack
- **Language:** C++20
- **Build System:** CMake
- **Graphics/Windowing:** SFML 3 (Strictly isolated to `src/renderer/` and `src/main.cpp`)

## 🎮 Running the Sandbox

You can build the sandbox using CMake. 

1. Ensure you have CMake and a C++20 compiler installed.
2. Ensure you have SFML 3 installed and accessible to CMake.
3. Build the project:
   ```bash
   cmake -S . -B build
   cmake --build build
   ```
4. Run the executable:
   ```bash
   ./build/PhysicsSandbox
   ```

### Controls
- **Left Click:** Spawn a physics circle.
- **Right Click:** Spawn a physics box.
- Watch as objects interact with gravity, the static boundaries, and the swinging pendulum joint!
