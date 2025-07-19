#pragma once
#include <SFML/Graphics.hpp>

class Node{

    private:


    public:
        Node() : x(-1), y(-1), parentX(-1), parentY(-1), gCost(0), hCost(0) {}
        Node(int x,int y): x(x), y(y), parentX(-1), parentY(-1), gCost(0), hCost(0){

            parent = sf::Vector2i(parentX, parentY);
        }

        int x,y; //localizacion del nodo
        sf::Vector2i parent;
        int parentX,parentY; //para reconstrucion del camino
        float gCost;//costo desde el inicio
        float hCost; //heuristica(adivinanza) hacial el final
        float fCost() const { return gCost + hCost; } //funcion de costo
        
        // Operador de comparación (para priority_queue)
        bool operator>(const Node& other) const {
            return this->fCost() > other.fCost();
        }


};