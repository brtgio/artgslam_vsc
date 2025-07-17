#pragma once
#include <cmath>
#include <queue>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/GridMap.hpp"
#include "artgslam_vsc/Node.hpp"


class AStar{

    private:
        enum class State{ 
            cStart,//celda de inicio
            cEmpty,//celda vacia
            cObstacle,//celda ocupada es decir obstaculo
            cClose, //celda ya evaluada
            cPath, //celda parte del camino final
            cGoal   //celda de destino (meta o goal en ingles)
        };

        //listas para open node y close node
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
        std::vector<std::vector<bool>> closedList;

        std::vector<sf::Vector2i> getNeighbors(const Node& node) const;

        std::vector<sf::Vector2i> reconstructPath(const Node& current);
        
        //variables para manejar el mapa
        std::vector<std::vector<State>> stateMap; //convierte al formato de estados derinido en State
        GridMap& map;
    
        int width, height; //largo y ancho de la cuadricula en nuestro caso son iguales ya que usamos rejilla simetrica

        sf::Vector2i startIndex,goalIndex;
        //heuristica (distancia euclidiana, razon: me dio la gana euclides esta guapo) 
        double euclideanHeuristic(int x1, int y1, int x2, int y2) {
            return std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
        }

        double octileHeuristic(int x1, int y1, int x2, int y2) {
            int dx = abs(x2 - x1);
            int dy = abs(y2 - y1);
            return std::max(dx, dy) + (std::sqrt(2) - 1) * std::min(dx, dy);
        }
        //verificamos si la celda esta dentro del mapa
        bool isValidCell (int x,int y) const {
            return x >= 0 && x<width && y >= 0 && y < height;
        }
    public:

        AStar(GridMap& mapRef);
    
        std::vector<sf::Vector2i> findPath(sf::Vector2i start, sf::Vector2i goal);
        std::vector<std::vector<State>> stateGrid;//vector para marcar celdas visitadas

};