#include <cmath>
#include <utility>
#include <cassert>
#include <iostream>

#include "particle_system.h"
#include "utils.h"



extern const sf::Vector2u WINDOW_SIZES;



Particle::Particle(
    const sf::Vector2f position, const sf::Vector2f velocity,
    const sf::Vector2f acceleration,
    const float radius, const float mass
)
    : position_(position)
    , velocity_(velocity)
    , acceleration_(acceleration)
    , radius_(radius)
    , mass_(mass)
{
    shape_.setRadius(radius);
    shape_.setFillColor(sf::Color::White);
    shape_.setOrigin(sf::Vector2f(radius, radius));
}



void Particle::Push(const sf::Vector2f delta_velocity)
{
    velocity_ += delta_velocity;
}



sf::Vector2f Particle::GetPosition() const
{
    return position_;
}



sf::Vector2f Particle::GetVelocity() const
{
    return velocity_;
}



sf::Vector2f Particle::GetAcceleration() const
{
    return acceleration_;
}



float Particle::GetRadius() const
{
    return radius_;
}



float Particle::GetMass() const
{
    return mass_;
}



void Particle::Update(const float dt)
{
    velocity_ += acceleration_ * dt;
    position_ += velocity_ * dt;

    if (position_.x < radius_)
    {
        position_.x = radius_;
        velocity_.x = -velocity_.x / 2.0f;
    }
    if (position_.x + radius_ > WINDOW_SIZES.x)
    {
        position_.x = WINDOW_SIZES.x - radius_;
        velocity_.x = -velocity_.x / 2.0f;
    }

    if (position_.y < radius_)
    {
        position_.y = radius_;
        velocity_.y = -velocity_.y / 2.0f;
    }
    const float bottom_gap = 50.0f;
    if (position_.y + radius_ > WINDOW_SIZES.y - bottom_gap)
    {
        position_.y = WINDOW_SIZES.y - radius_ - bottom_gap;
        velocity_.y = -velocity_.y / 2.0f;
    }
}



void Particle::Render(sf::RenderWindow& window)
{
    shape_.setPosition(position_);
    shape_.setRadius(radius_);

    window.draw(shape_);
}



ParticleSystem::ParticleID ParticleSystem::AddParticle(
    sf::Vector2f position, sf::Vector2f velocity,
    sf::Vector2f acceleration,
    const float radius, const float mass)
{
    const ParticleID new_particle_id = particles_.size();
    particles_.emplace_back(Particle(
        position, velocity, acceleration, radius, mass
    ));
    return new_particle_id;
}



void ParticleSystem::AddLink(
    const ParticleID particle1_id, const ParticleID particle2_id,
    const float stiffness)
{
    links_.push_back(Link{
        particle1_id, particle2_id,
        GetDistance(particle1_id, particle2_id), stiffness
    });
}



const Particle& ParticleSystem::GetParticleByID(
    const ParticleID particle_id) const
{
    //assert(0 <= particle_id < particles_.size());
    return particles_[particle_id];
}



float ParticleSystem::GetDistance(
    const ParticleID particle1_id, const ParticleID particle2_id) const
{
    const Particle& particle1 = GetParticleByID(particle1_id);
    const Particle& particle2 = GetParticleByID(particle2_id);

    const sf::Vector2f delta = (
        particle1.GetPosition() - particle2.GetPosition()
    );
    return ComputeLength(delta);
}



void ParticleSystem::Push(const sf::Vector2f velocity)
{
    for (size_t i = 0; i < particles_.size(); i++)
    {
        particles_[i].Push(velocity);
    }
}



void ParticleSystem::Update(const float dt)
{
    for (size_t i = 0; i < particles_.size(); i++)
    {
        particles_[i].acceleration_ = sf::Vector2f(0, 0);
    }

    SolveLinks();
    ApplyGravity();

    for (size_t i = 0; i < particles_.size(); i++)
    {
        particles_[i].Update(dt);
    }

    HandleCollisions();
}



void ParticleSystem::Render(sf::RenderWindow& window)
{
    for (size_t i = 0; i < particles_.size(); i++)
    {
        particles_[i].Render(window);
    }
}



void ParticleSystem::ApplyGravity()
{
    const sf::Vector2f acceleration_g(0, 500.0f);

    for (size_t i = 0; i < particles_.size(); i++)
    {
        particles_[i].acceleration_ += acceleration_g;
    }
}



void ParticleSystem::SolveLinks()
{
    for (size_t i = 0; i < links_.size(); i++)
    {
        Particle& particle1 = particles_[links_[i].particle1_id];
        Particle& particle2 = particles_[links_[i].particle2_id];
        const sf::Vector2f distance_vector12 = (
            particle2.GetPosition() - particle1.GetPosition()
        );
        const float distance = ComputeLength(distance_vector12);
        const float delta = distance - links_[i].intitial_distance;
        const float force = links_[i].stiffness * delta;

        const sf::Vector2f unit_distance_vector12 = (
            distance_vector12 / ComputeLength(distance_vector12)
        );
        const sf::Vector2f particle1_acceleration = (
            unit_distance_vector12 * force / particle1.GetMass()
        );
        const sf::Vector2f particle2_acceleration = (
            -unit_distance_vector12 * force / particle2.GetMass()
        );
        particle1.acceleration_ += particle1_acceleration;
        particle2.acceleration_ += particle2_acceleration;
    }
}



void ParticleSystem::HandleCollisions()
{
    for (size_t i = 0; i < particles_.size(); i++)
    {
        for (size_t j = 0; j < particles_.size(); j++)
        {
            if (i != j)
            {
                HandleCollisionBetween(particles_[i], particles_[j]);
            }
        }
    }
}



void ParticleSystem::HandleCollisionBetween(
    Particle& particle1, Particle& particle2)
{
    const sf::Vector2f distance_vector12 = (
        particle2.GetPosition() - particle1.GetPosition()
    );
    const float distance = ComputeLength(distance_vector12);
    const float min_distance = particle1.GetRadius() + particle2.GetRadius();
    if (distance >= min_distance)
    {
        return;
    }

    const sf::Vector2f unit_distance_vector12 = distance_vector12 / distance;
    particle1.position_ += -unit_distance_vector12 * (min_distance - distance);
    particle2.position_ += unit_distance_vector12 * (min_distance - distance);
}


