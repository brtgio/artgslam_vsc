#pragma once

#include <cmath>
#include <queue>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/GridMap.hpp"
#include "artgslam_vsc/Node.hpp"

class AStar {
public:
    enum class State {
        cStart,
        cEmpty,
        cObstacle,
        cClose,
        cPath,
        cGoal
    };

    AStar(GridMap& mapRef);

    // Control del algoritmo
    void start();
    bool step();
    std::vector<sf::Vector2i> findPath();
    void updatemap();

    // Visualización y estado
    void draw(sf::RenderTarget& target, float pixelsPerMeter, float metersPerCell) const;
    void drawFoundPath(sf::RenderTarget& target, float pixelsPerMeter, float metersPerCell) const;
    bool isFinished() const { return finished; }
    bool isPathFound() const { return pathFound; }
    const Node& getCurrentNode() const { return currentNode; }
    const std::vector<std::vector<State>>& getStateMap() const { return stateMap; }

private:
    GridMap& map;

    // Estados internos
    int width = 0;
    int height = 0;
    bool pathFound = false;
    bool finished = false;

    // Coordenadas del algoritmo
    sf::Vector2i startIndex = {-1, -1};
    sf::Vector2i goalIndex  = {-1, -1};
    Node currentNode;
    Node goalNode;

    // Estructuras del algoritmo
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::vector<std::vector<bool>> closedList;
    std::vector<std::vector<float>> gScore;
    std::vector<std::vector<sf::Vector2i>> parentMap;
    std::vector<std::vector<State>> stateMap;
    std::vector<sf::Vector2i> finalPath;

    // Métodos auxiliares
    double euclideanHeuristic(int x1, int y1, int x2, int y2);
    double octileHeuristic(int x1, int y1, int x2, int y2) {
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
    }

    bool isValidCell(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    std::vector<sf::Vector2i> getNeighbors(const Node& node) const;
    std::vector<sf::Vector2i> reconstructPath(const Node& endNode) const;
};
