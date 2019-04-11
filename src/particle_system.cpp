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



void Particle::Push(const sf::Vector2f force)
{
    acceleration_ += force / mass_;
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
    const float min_spring_length = 20.0f;

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
    // assert(0 <= particle_id && particle_id < particles_.size());
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
    return utils::ComputeLength(delta);
}



void ParticleSystem::Push(const sf::Vector2f force)
{
    for (Particle& particle : particles_)
    {
        particle.Push(force);
    }
}



void ParticleSystem::Update(const float dt)
{
    SolveLinks();
    ApplyGravity();

    for (Particle& particle : particles_)
    {
        particle.Update(dt);
        particle.acceleration_ = sf::Vector2f(0.0f, 0.0f);
    }

    HandleCollisionsBetweenParticles();
    HandleCollisionsWithWalls();
}



void ParticleSystem::Render(sf::RenderWindow& window)
{
    const float max_spring_width = 18.0f;
    sf::VertexArray spring(sf::Quads, 4);

    for (const Link& link : links_)
    {
        const Particle& particle1 = particles_[link.particle1_id];
        const Particle& particle2 = particles_[link.particle2_id];
        const sf::Vector2f center1 = particle1.GetPosition();
        const sf::Vector2f center2 = particle2.GetPosition();
        const sf::Vector2f distance_vector12 = center2 - center1;
        const float spring_length = (utils::ComputeLength(distance_vector12)
            - particle1.GetRadius() - particle2.GetRadius()
        );
        const float spring_width = (max_spring_width
            * 10.0f / (9.0f + spring_length / link.min_length)
        );
        const sf::Vector2f ort_unit_vector = (
            utils::GetOrthogonalVector(distance_vector12) /
            utils::ComputeLength(distance_vector12)
        );
        spring[0].position = center1 + ort_unit_vector * spring_width / 2.0f;
        spring[1].position = center1 - ort_unit_vector * spring_width / 2.0f;
        spring[2].position = center2 - ort_unit_vector * spring_width / 2.0f;
        spring[3].position = center2 + ort_unit_vector * spring_width / 2.0f;

        const float stretch = spring_length / link.min_length;
        const float red_color = stretch < 16.0f ? stretch * 16.0f : 255.0f;
        spring[0].color = sf::Color(sf::Uint8(red_color), 96, 0);
        spring[1].color = sf::Color(sf::Uint8(red_color), 96, 0);
        spring[2].color = sf::Color(sf::Uint8(red_color), 96, 0);
        spring[3].color = sf::Color(sf::Uint8(red_color), 96, 0);

        window.draw(spring);
    }

    for (Particle& particle : particles_)
    {
        particle.Render(window);
    }
}



void ParticleSystem::ApplyGravity()
{
    const sf::Vector2f acceleration_g(0, 980.0f);
    for (Particle& particle : particles_)
    {
        particle.acceleration_ += acceleration_g;
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
        const float distance = utils::ComputeLength(distance_vector12);
        const float delta = distance - link.intitial_distance;
        const float force = link.stiffness * delta;
        const sf::Vector2f unit_vector12 = (
            distance_vector12 / utils::ComputeLength(distance_vector12)
        );
        particle1.acceleration_ += unit_vector12 * force / particle1.GetMass();
        particle2.acceleration_ -= unit_vector12 * force / particle2.GetMass();

        const float spring_length = (
            distance - particle1.GetRadius() - particle2.GetRadius()
        );

        if (spring_length < link.min_length)
        {
            const float velocity_reduce_factor = sqrt(2.0f);
            particle1.velocity_ = utils::Reflect(
                particle1.GetVelocity(), -distance_vector12
            ) / velocity_reduce_factor;
            particle2.velocity_ = utils::Reflect(
                particle2.GetVelocity(), distance_vector12
            ) / velocity_reduce_factor;

            const sf::Vector2f position_correction = (
                unit_vector12 * (link.min_length - spring_length) / 2.0f
            );
            particle1.position_ += -position_correction;
            particle2.position_ +=  position_correction;
        }
    }
}



void ParticleSystem::HandleCollisionsBetweenParticles()
{
    std::vector<bool> is_velocity_updated(particles_.size(), false);
    std::vector<sf::Vector2f> new_velocities(
        particles_.size(), sf::Vector2f(0, 0)
    );

    for (size_t i = 0; i < particles_.size(); i++)
    {
        for (size_t j = 0; j < particles_.size(); j++)
        {
            const sf::Vector2f vector_ij = (
                particles_[j].GetPosition() - particles_[i].GetPosition()
            );
            const float distance = utils::ComputeLength(vector_ij);
            const float min_distance = (
                particles_[i].GetRadius() + particles_[j].GetRadius()
            );
            if (i >= j || distance >= min_distance)
            {
                continue;
            }

            const auto solved_collision_info = SolveCollisionBetween(
                particles_[i], particles_[j]
            );
            new_velocities[i] += solved_collision_info.particle1_new_velocity;
            new_velocities[j] += solved_collision_info.particle2_new_velocity;
            is_velocity_updated[i] = true;
            is_velocity_updated[j] = true;
        }
    }

    // for (size_t i = 0; i < particles_.size(); i++)
    // {
    //     if (is_velocity_updated[i])
    //     {
    //         particles_[i].velocity_ = new_velocities[i];
    //     }
    // }
}



ParticleSystem::SolvedCollisionInfo ParticleSystem::SolveCollisionBetween(
    const Particle& particle1, const Particle& particle2) const
{
    // TODO: solve collision equations between two particles
    return {particle1.GetVelocity(), particle2.GetVelocity()};
}



void ParticleSystem::HandleCollisionsWithWalls()
{
    const float left_border = 0.0f;
    const float right_border = WINDOW_SIZES.x - 0.0f;
    const float up_border = 0.0f;
    const float bottom_border = WINDOW_SIZES.y - 0.0f;
    const float velocity_reduce_factor = sqrt(2.0f);

    for (Particle& particle : particles_)
    {
        if (particle.position_.x < particle.radius_ + left_border)
        {
            particle.position_.x = particle.radius_ + left_border;
            particle.velocity_.x = -particle.velocity_.x /
                                   velocity_reduce_factor;
        }

        if (particle.position_.x + particle.radius_ > right_border)
        {
            particle.position_.x = right_border - particle.radius_;
            particle.velocity_.x = -particle.velocity_.x /
                                   velocity_reduce_factor;
        }

        if (particle.position_.y < particle.radius_ + up_border)
        {
            particle.position_.y = particle.radius_ + up_border;
            particle.velocity_.y = -particle.velocity_.y /
                                   velocity_reduce_factor;
        }

        if (particle.position_.y + particle.radius_ > bottom_border)
        {
            particle.position_.y = bottom_border - particle.radius_;
            particle.velocity_.y = -particle.velocity_.y /
                                   velocity_reduce_factor;
        }
    }   
}

