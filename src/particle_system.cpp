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
    const float min_spring_length = 100.0f;

    links_.push_back(Link{
        particle1_id, particle2_id,
        GetDistance(particle1_id, particle2_id),
        stiffness,
        min_spring_length
    });
}



const Particle& ParticleSystem::GetParticleByID(
    const ParticleID particle_id) const
{
    //assert(0 <= particle_id && particle_id < particles_.size());
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

    //HandleCollisionsBetweenParticles();
    HandleCollisionsWithWalls();
}



void ParticleSystem::Render(sf::RenderWindow& window)
{
    for (const Link& link : links_)
    {
        sf::Vertex spring[2] = {
            sf::Vertex(GetParticleByID(link.particle1_id).GetPosition()),
            sf::Vertex(GetParticleByID(link.particle2_id).GetPosition())
        };
        spring[0].color = sf::Color(64, 64, 64);
        spring[1].color = sf::Color(64, 64, 64);
        window.draw(spring, 2, sf::Lines);
    }

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
    for (const Link& link : links_)
    {
        Particle& particle1 = particles_[link.particle1_id];
        Particle& particle2 = particles_[link.particle2_id];
        const sf::Vector2f distance_vector12 = (
            particle2.GetPosition() - particle1.GetPosition()
        );
        const float distance = ComputeLength(distance_vector12);
        const float delta = distance - link.intitial_distance;
        const float force = link.stiffness * delta;

        const sf::Vector2f unit_vector12 = (
            distance_vector12 / ComputeLength(distance_vector12)
        );
        const sf::Vector2f particle1_acceleration = (
            unit_vector12 * (force / particle1.GetMass())
        );
        const sf::Vector2f particle2_acceleration = (
            -unit_vector12 * (force / particle2.GetMass())
        );
        particle1.acceleration_ += particle1_acceleration;
        particle2.acceleration_ += particle2_acceleration;

        const float spring_length = (
            distance - particle1.GetRadius() - particle2.GetRadius()
        );
        
        if (spring_length < link.min_length)
        {
            particle1.velocity_ = sf::Vector2f(0, 0);
            particle2.velocity_ = sf::Vector2f(0, 0);

            const sf::Vector2f position_correction = (
                unit_vector12 * (link.min_length - spring_length) / 2.0f
            );
            //std::cout << "==========================================\n";
            //std::cout << "particle1 before: pos.x = " << particle1.GetPosition().x << "  pos.y = " << particle1.GetPosition().y << "  radius = " << particle1.GetRadius() << "\n";
            //std::cout << "particle2 before: pos.x = " << particle2.GetPosition().x << "  pos.y = " << particle2.GetPosition().y << "  radius = " << particle2.GetRadius() << "\n";
            //std::cout << "spring_length = " << spring_length << std::endl;
            //std::cout << "correction: x = " << correction.x << "  y = " << correction.y << std::endl;

            particle1.position_ += -position_correction;
            particle2.position_ +=  position_correction;

            //std::cout << "particle1 after: pos.x = " << particle1.GetPosition().x << "  pos.y = " << particle1.GetPosition().y << "  radius = " << particle1.GetRadius() << "\n";
            //std::cout << "particle2 after: pos.x = " << particle2.GetPosition().x << "  pos.y = " << particle2.GetPosition().y << "  radius = " << particle2.GetRadius() << "\n";
            //std::cout << "==========================================\n";
        }
    }
}



void ParticleSystem::HandleCollisionsBetweenParticles()
{
    std::vector<sf::Vector2f> corrections(
        particles_.size(), sf::Vector2f(0, 0)
    );

    for (size_t i = 0; i < particles_.size(); i++)
    {
        for (size_t j = 0; j < particles_.size(); j++)
        {
            const sf::Vector2f vector_ij = (
                particles_[j].GetPosition() - particles_[i].GetPosition()
            );
            const float distance = ComputeLength(vector_ij);
            const float min_distance = (
                particles_[i].GetRadius() + particles_[j].GetRadius()
            );
            if (i >= j || distance >= min_distance)
            {
                continue;
            }

            const sf::Vector2f unit_vector_ij = vector_ij / distance;
            corrections[i] += -unit_vector_ij * (min_distance - distance) / 2.0f;
            corrections[j] +=  unit_vector_ij * (min_distance - distance) / 2.0f;

            /*std::cout << "========================================================\n";
            std::cout << "particle[i]: pos.x = " << particles_[i].GetPosition().x << "  pos.y = " << particles_[i].GetPosition().y << "  radius = " << particles_[i].GetRadius() << "\n";
            std::cout << "particle[j]: pos.x = " << particles_[j].GetPosition().x << "  pos.y = " << particles_[j].GetPosition().y << "  radius = " << particles_[j].GetRadius() << "\n";
            std::cout << "min_distance - distance = " << min_distance - distance << std::endl;
            std::cout << "i = " << i << " correction = " << corrections[i].y << std::endl;
            std::cout << "j = " << j << " correction = " << corrections[j].y << std::endl;
            std::cout << "========================================================\n";*/
        }
    }

    for (size_t i = 0; i < particles_.size(); i++)
    {
        if (corrections[i] != sf::Vector2f(0, 0))
        {
            particles_[i].position_ += corrections[i];
            const float abs_velocity = ComputeLength(particles_[i].velocity_);
            particles_[i].velocity_ = abs_velocity * (
                corrections[i] / ComputeLength(corrections[i])
            );
        }
    }
}



void ParticleSystem::HandleCollisionsWithWalls()
{
    const float up_gap = 0.0f;
    const float right_gap = 0.0f;
    const float bottom_gap = 0.0f;
    const float left_gap = 0.0f;
    const float velocity_reduce_factor = sqrt(2.0f);

    for (Particle& particle : particles_)
    {
        const float left_border = left_gap;
        if (particle.position_.x < particle.radius_ + left_border)
        {
            particle.position_.x = particle.radius_ + left_border;
            particle.velocity_.x = -particle.velocity_.x /
                                   velocity_reduce_factor;
        }

        const float right_border = WINDOW_SIZES.x - right_gap;
        if (particle.position_.x + particle.radius_ > right_border)
        {
            particle.position_.x = right_border - particle.radius_;
            particle.velocity_.x = -particle.velocity_.x /
                                   velocity_reduce_factor;
        }

        const float up_border = up_gap;
        if (particle.position_.y < particle.radius_ + up_border)
        {
            particle.position_.y = particle.radius_ + up_border;
            particle.velocity_.y = -particle.velocity_.y /
                                   velocity_reduce_factor;
        }

        const float bottom_border = WINDOW_SIZES.y - bottom_gap;
        if (particle.position_.y + particle.radius_ > bottom_border)
        {
            particle.position_.y = bottom_border - particle.radius_;
            particle.velocity_.y = -particle.velocity_.y /
                                   velocity_reduce_factor;
        }
    }   
}

