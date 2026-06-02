#include <SFML/Graphics.hpp>
#include <optional>

int main()
{
    sf::RenderWindow window(
        sf::VideoMode({800, 600}),
        "Physics Sandbox"
    );

    sf::CircleShape circle(50.f);
    circle.setPosition({300.f, 200.f});

    // velocity vector
    sf::Vector2f velocity(100.f, 100.f);

    // clock for delta time
    sf::Clock clock;

    while (window.isOpen())
    {
        float dt = clock.restart().asSeconds();

        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
        }

        // get current position
        sf::Vector2f position = circle.getPosition();

        // movement
        position += velocity * dt;

        // update circle position
        circle.setPosition(position);

        window.clear();

        window.draw(circle);

        window.display();
    }

    return 0;
}