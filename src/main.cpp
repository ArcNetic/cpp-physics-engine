#include <SFML/Graphics.hpp>
#include <optional>

int main()
{
    sf::RenderWindow window(
        sf::VideoMode({800, 600}),
        "Physics Sandbox"
    );

    // Creating a new Circle
    sf::CircleShape circle(50.f);
    circle.setPosition({300.f, 200.f});

    // Velocity vector
    sf::Vector2f velocity(0.f, 0.f);

    // Gravity Acceleration
    const float gravity = 980.f;

    // clock for dt
    sf::Clock clock;

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

        // update
        circle.setPosition(position);

        // render
        window.clear();

        window.draw(circle);

        window.display();
    }

    return 0;
}