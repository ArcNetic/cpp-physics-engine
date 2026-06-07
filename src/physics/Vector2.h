#pragma once

namespace Physics
{
    struct Vector2
    {
        float x;
        float y;

        Vector2() : x(0.f), y(0.f) {}
        Vector2(float x, float y) : x(x), y(y) {}

        Vector2 operator+(const Vector2& o) const { return {x + o.x, y + o.y}; }
        Vector2 operator-(const Vector2& o) const { return {x - o.x, y - o.y}; }
        Vector2 operator*(float s) const { return {x * s, y * s}; }
        Vector2 operator/(float s) const { return {x / s, y / s}; }
        Vector2 operator-() const { return {-x, -y}; }

        Vector2& operator+=(const Vector2& o) { x += o.x; y += o.y; return *this; }
        Vector2& operator-=(const Vector2& o) { x -= o.x; y -= o.y; return *this; }
        Vector2& operator*=(float s) { x *= s; y *= s; return *this; }
        Vector2& operator/=(float s) { x /= s; y /= s; return *this; }
        
        bool operator==(const Vector2& o) const { return x == o.x && y == o.y; }
        bool operator!=(const Vector2& o) const { return x != o.x || y != o.y; }
    };

    inline Vector2 operator*(float s, const Vector2& v) { return {v.x * s, v.y * s}; }
}

