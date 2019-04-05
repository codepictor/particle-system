#include "game.h"



extern const sf::Vector2u WINDOW_SIZES(800, 600);



Game::Game()
    : main_window_("Particle system", WINDOW_SIZES)
{
    ParticleSystem::ParticleID particle1_id = particle_system_.AddParticle(
        sf::Vector2f(100.0f, 100.0f),  // position
        sf::Vector2f(0, 0),  // velocity
        sf::Vector2f(0, 0),  // acceleration
        25.0f,  // radius
        15000.0f  // mass
    );

    ParticleSystem::ParticleID particle2_id = particle_system_.AddParticle(
        sf::Vector2f(100.0f, 400.0f),  // position
        sf::Vector2f(0, 0),  // velocity
        sf::Vector2f(0, 0),  // acceleration
        25.0f,  // radius
        15000.0f  // mass
    );

    /*ParticleSystem::ParticleID particle3_id = particle_system_.AddParticle(
        sf::Vector2f(400.0f, 100.0f),  // position
        sf::Vector2f(0, 0),  // velocity
        sf::Vector2f(0, 0),  // acceleration
        25.0f,  // radius
        15000.0f  // mass
    );

    ParticleSystem::ParticleID particle4_id = particle_system_.AddParticle(
        sf::Vector2f(400.0f, 400.0f),  // position
        sf::Vector2f(0, 0),  // velocity
        sf::Vector2f(0, 0),  // acceleration
        25.0f,  // radius
        15000.0f  // mass
    );*/

    particle_system_.AddLink(
        particle1_id, particle2_id,
        1e5,  // stiffness
        10.0f  // min_length
    );

    /*particle_system_.AddLink(
        particle2_id, particle3_id,
        1e5,  // stiffness
        10.0f  // min_length
    );

    particle_system_.AddLink(
        particle3_id, particle4_id,
        1e5,  // stiffness
        10.0f  // min_length
    );

    particle_system_.AddLink(
        particle4_id, particle1_id,
        1e5,  // stiffness
        10.0f  // min_length
    );*/
}



void Game::Run()
{
    const float updates_per_second = 60.0f;
    const sf::Time time_per_update = sf::seconds(1.0f / updates_per_second);

    sf::Clock clock;
    sf::Time time_since_last_update = sf::Time::Zero;

    while (!main_window_.CheckIsDone())
    {
        time_since_last_update += clock.restart();
        while (time_since_last_update >= time_per_update)
        {
            HandleInput();
            time_since_last_update -= time_per_update;
            Update(time_per_update.asSeconds());
        }

        Render();
    }
}



void Game::HandleInput()
{
    sf::Vector2f velocity(0, 0);
    const float delta = 20.0f;

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
    {
        velocity.y -= delta;
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
    {
        velocity.x += delta;
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
    {
        velocity.y += delta;
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
    {
        velocity.x -= delta;
    }

    particle_system_.Push(velocity);
}



void Game::Update(const float dt)
{
    main_window_.Update();

    particle_system_.Update(dt);
}



void Game::Render()
{
    main_window_.BeginDraw();

    main_window_.Draw(particle_system_);
    // draw other objects here

    main_window_.EndDraw();
}


