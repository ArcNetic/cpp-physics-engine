cmake_minimum_required(VERSION 3.16)

project(PhysicsSandbox)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML COMPONENTS
    Graphics
    Window
    System
    REQUIRED
)

add_executable(PhysicsSandbox
    src/main.cpp
    src/physics/Ball.cpp
)

target_link_libraries(PhysicsSandbox PRIVATE
    SFML::Graphics
    SFML::Window
    SFML::System
)