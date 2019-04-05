#pragma once

#include <SFML/Graphics.hpp>



namespace utils
{


float ComputeLength(const sf::Vector2f vector);

sf::Vector2f GetOrthogonalVector(const sf::Vector2f original_vector);
sf::Vector2f Reflect(const sf::Vector2f vector, const sf::Vector2f mirror);
sf::Vector2f GetProjection(
    const sf::Vector2f vector, const sf::Vector2f direction
);


}  // namespace utils

