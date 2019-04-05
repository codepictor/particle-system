#include <cmath>

#include "utils.h"



namespace utils
{


float ComputeLength(const sf::Vector2f vector)
{
    return sqrt(vector.x * vector.x + vector.y * vector.y);
}



sf::Vector2f GetOrthogonalVector(const sf::Vector2f original_vector)
{
    return sf::Vector2f(
        -original_vector.y,
         original_vector.x
    );
}



sf::Vector2f Reflect(const sf::Vector2f vector, const sf::Vector2f mirror)
{
    const sf::Vector2f orthogonal_mirror = GetOrthogonalVector(mirror);
    const sf::Vector2f vector_projection_on_mirror = GetProjection(
        vector, orthogonal_mirror
    );
    return 2.0f * vector_projection_on_mirror - vector;
}



sf::Vector2f GetProjection(
    const sf::Vector2f vector, const sf::Vector2f direction)
{
    const float scalar_product = (
        vector.x * direction.x + vector.y * direction.y
    );

    return (
        direction / ComputeLength(direction) *
        scalar_product / ComputeLength(direction)
    );
}


}  // namespace utils
