#include "UnicicleWmr.hpp"

UnicicleWmr::UnicicleWmr(float width_m, float length_m, float pixelsPerMeter)
: width(width_m), length(length_m), pixelsPerMeter(pixelsPerMeter),
    x(0.f), y(0.f), theta(0.f),
    linear_v(0.f), angular_omega(0.f)
{
    robotShape.setSize(sf::Vector2f(length * pixelsPerMeter, width * pixelsPerMeter));
    robotShape.setOrigin(robotShape.getSize().x / 2.f, robotShape.getSize().y / 2.f);
    robotShape.setFillColor(sf::Color::Blue);
}

void UnicicleWmr::update(float dt)
{
}

void UnicicleWmr::draw(sf::RenderWindow &window)
{
    robotShape.setPosition(x,y);
    robotShape.setRotation(theta * 180.f / static_cast<float>(PI)); // Radianes → grados
    window.draw(robotShape);
}

void UnicicleWmr::setVelocity(float linear_v, float angular_omega)
{
   this-> linear_v = linear_v;
    this->angular_omega = angular_omega;

}

void UnicicleWmr::reset(float x, float y, float theta)
{
    setPose(x, y, theta);
    setVelocity(0.f, 0.f);
}

void UnicicleWmr::setPose(float x, float y, float theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}
