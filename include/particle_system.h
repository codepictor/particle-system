#pragma once

#include <vector>

#include <SFML/Graphics.hpp>



class Particle
{
    friend class ParticleSystem;

public:
    Particle(
        const sf::Vector2f position, const sf::Vector2f velocity,
        const sf::Vector2f acceleration,
        const float radius, const float mass
    );
    virtual ~Particle() = default;

    sf::Vector2f GetPosition() const;
    sf::Vector2f GetVelocity() const;
    sf::Vector2f GetAcceleration() const;
    float GetRadius() const;
    float GetMass() const;

    void Push(const sf::Vector2f delta_velocity);

    void Update(const float dt);
    void Render(sf::RenderWindow& window);


private:
    sf::Vector2f position_;
    sf::Vector2f velocity_;
    sf::Vector2f acceleration_;
    float radius_;
    float mass_;

    sf::CircleShape shape_;
};



class ParticleSystem
{
public:
    ParticleSystem() = default;
    virtual ~ParticleSystem() = default;

    using ParticleID = size_t;

    ParticleID AddParticle(
        const sf::Vector2f position, const sf::Vector2f velocity,
        const sf::Vector2f acceleration,
        const float radius, const float mass
    );
    void AddLink(
        const ParticleID particle1_id, const ParticleID particle2_id,
        const float stiffness
    );

    const Particle& GetParticleByID(const ParticleID particle_id) const;
    float GetDistance(
        const ParticleID particle1_id, const ParticleID particle2_id
    ) const;

    void Push(const sf::Vector2f delta_velocity);

    void Update(const float dt);
    void Render(sf::RenderWindow& window);


private:
    struct Link
    {
        ParticleID particle1_id;
        ParticleID particle2_id;
        float intitial_distance;
        float stiffness;
        float min_length;
    };

    void SolveLinks();
    void ApplyGravity();
    void HandleCollisionsBetweenParticles();
    void HandleCollisionsWithWalls();

    struct SolvedCollisionInfo
    {
        sf::Vector2f particle1_new_velocity;
        sf::Vector2f particle2_new_velocity;
    };

    SolvedCollisionInfo SolveCollisionBetween(
        const Particle& particle1, const Particle& particle2
    ) const;

    std::vector<Particle> particles_;
    std::vector<Link> links_;
};

