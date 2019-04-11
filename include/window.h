#pragma once

#include <string>
#include <SFML/Graphics.hpp>

#include "particle_system.h"



class Window
{
public:
    Window(const std::string& title, const sf::Vector2u& size);
    virtual ~Window();

    bool CheckIsDone();

    void Update();
    void BeginDraw();
    void Draw(sf::Drawable& drawable_object);
    void Draw(Particle& particle);
    void Draw(ParticleSystem& particle_system);
    void EndDraw();
    

private:
    void Create();
    void Destroy();

    sf::RenderWindow window_;
    bool is_done_ = false;
    std::string default_title_;
    sf::Vector2u default_size_;
};

