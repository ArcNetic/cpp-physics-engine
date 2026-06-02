#include <SFML/Graphics.hpp>
#include <optional>

int main()
{
    sf::RenderWindow window(
        sf::VideoMode({800, 600}),
        "Physics Sandbox");

    // Creating a new Circle
    float radius = 50.f;
    sf::CircleShape circle(radius);
    circle.setPosition({300.f, 200.f});

    // Velocity vector
    sf::Vector2f velocity(0.f, 0.f);

    // Floor
    const float floorY = 500.f;

    // Gravity Acceleration
    const float gravity = 980.f;

    //Co-efficient of restituion
    const float e = 0.7f;

    // clock for dt
    sf::Clock clock;

    // Draw floor
    sf::RectangleShape floor(sf::Vector2f({800.f, 20.f}));
    floor.setPosition({0.f, 550.f});
    floor.setFillColor(sf::Color::White);

    while (window.isOpen())
    {
        float dt = clock.restart().asSeconds(); // time since the last frame

        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
        }

        // Apply gravity
        velocity.y += gravity * dt;

        // Get current position
        sf::Vector2f position = circle.getPosition();

        // Movement
        position += velocity * dt;

        // Collision
        if (position.y + radius >= floorY)
        {
            position.y = floorY - radius;

            // Stop movement
            velocity.y = (0 - velocity.y) * e;
        }

        // update
        circle.setPosition(position);

        // render
        window.clear();

        window.draw(circle);
        window.draw(floor);
        window.display();
    }

    return 0;
}