#include "UnicicleWmr.hpp"

UnicicleWmr::UnicicleWmr(float width_m = 0.28f, float length_m = 0.33f)
{
}

void UnicicleWmr::update(float dt)
{
}

void UnicicleWmr::draw(sf::RenderWindow &window)
{
    robotShape.setPosition(x,y);
    robotShape.setRotation(theta * 180.f / static_cast<float>(PI)); // Radianes → grados
}

void UnicicleWmr::setVelocity(float line_v, float angular_omega)
{
}

void UnicicleWmr::reset(float x, float y, float theta)
{
}
