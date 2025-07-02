#pragma once
#include <SFML/Graphics.hpp>
#include "MyConstants.hpp"
class UnicicleWmr{

    public:
        UnicicleWmr(float width_m = 0.28f, float length_m = 0.33f, float pixelsPerMeter = 50.0f);

        void update(float dt);
        void draw(sf::RenderWindow& window);
        void setVelocity(float line_v,float angular_omega);
        void reset(float x= 0.0f , float y = 0.0f , float theta = 0.0f );

        //Obtener parametros del robot
        float getX() const { return x; }
        float getY() const { return y; }
        float getTheta() const { return theta; }

        float getWidth() const { return width; }
        float getLength() const { return length; }

        void setPose(float x,float y,float theta);

    private:

        float pixelsPerMeter;
        //Dimenciones
        float width;
        float length;

        //Pose
        float x,y,theta;
        
        //velocidad
        float linear_v;
        float angular_omega;

        //Grafico del carro
        sf::RectangleShape robotShape;

};