#include "artgslam_vsc/UnicicleWmr.hpp"

UnicicleWmr::UnicicleWmr(float width_m, float height_m, float pixelsPerMeter)
    : width(width_m), height(height_m), pixelsPerMeter(pixelsPerMeter),
      x(0.f), y(0.f), theta(0.f),
      linear_v(0.f), angular_omega(0.f),
      robotcolor(sf::Color::Blue)
{
    robotShape.setSize(sf::Vector2f(width * pixelsPerMeter, height * pixelsPerMeter));
    robotShape.setOrigin(robotShape.getSize() / 2.f);
    robotShape.setFillColor(robotcolor);
}

void UnicicleWmr::update(float dt)
{
    x += linear_v * cos(theta) * dt;
    y += linear_v * sin(theta) * dt;
    theta += angular_omega * dt;

}

void UnicicleWmr::draw(sf::RenderWindow& window)
{
    if (isRobotActive==true){
        robotShape.setPosition(x * pixelsPerMeter, y * pixelsPerMeter);
        robotShape.setRotation(theta * 180.f / 3.14159265f);  // De radianes a grados
        robotShape.setFillColor(robotcolor);
        window.draw(robotShape);
    }

}

void UnicicleWmr::setVelocity(float linear_v, float angular_omega)
{
    this->linear_v = linear_v;
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

void UnicicleWmr::setColor(sf::Color color)
{
    robotcolor = color;
}

void UnicicleWmr::setDimensions(float width, float height)
{
    this->width = width;
    this->height = height;
    robotShape.setSize(sf::Vector2f(width * pixelsPerMeter, height * pixelsPerMeter));
    robotShape.setOrigin(robotShape.getSize() / 2.f);
}
