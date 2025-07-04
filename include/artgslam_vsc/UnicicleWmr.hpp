#pragma once
#include <SFML/Graphics.hpp>
#include "MyConstants.hpp"
#include <math.h>
class UnicicleWmr{

    public:
        UnicicleWmr(float width_m = 0.28f, float height_m = 0.33f, float pixelsPerMeter = 50.0f);

        void update(float dt);
        void draw(sf::RenderWindow& window);
        void setVelocity(float line_v,float angular_omega);
        void reset(float x= 0.0f , float y = 0.0f , float theta = 0.0f );

        //Obtener parametros del robot
        float getX() const { return x; }
        float getY() const { return y; }
        float getTheta() const { return theta; }

        float getWidth() const { return width; }
        float getheight() const { return height; }
        void setRobotActive(bool active) { isRobotActive = active; }
        bool getRobotActive() const {return isRobotActive;};

        void setPose(float x,float y,float theta);
        void setColor(sf::Color color);
        void setDimensions(float width, float height);

    private:
        
        float pixelsPerMeter;
        //Dimenciones
        float width;
        float height;

        //Pose
        float x,y,theta;
        
        //velocidad
        float linear_v;
        float angular_omega;

        //Grafico del carro
        sf::RectangleShape robotShape;
        sf::Color robotcolor;

        //robot activity
        bool isRobotActive = false;

};