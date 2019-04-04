#pragma once

#include "window.h"
#include "particle_system.h"



class Game
{
public:
    Game();
    virtual ~Game() = default;
    
    void Run();

    void HandleInput();
    void Update(const float dt);
    void Render();

    int abc = 19;
private:
    ParticleSystem particle_system_;

    Window main_window_;
};

