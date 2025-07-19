#include "artgslam_vsc/AStar.hpp"
#include <iostream>
#include <cmath>
#include <algorithm> // std::reverse
#include <limits>


AStar::AStar(GridMap& mapRef)
    : map(mapRef)
{
    width = map.getMapSize();
    height = width;

    stateMap.resize(height, std::vector<State>(width, State::cEmpty));
    closedList.resize(height, std::vector<bool>(width, false));
    parentMap.resize(height, std::vector<sf::Vector2i>(width, {-1, -1}));
    gScore.resize(height, std::vector<float>(width, std::numeric_limits<float>::infinity()));

    startIndex = {-1, -1};
    goalIndex = {-1, -1};

    pathFound = false;
    finished = false;
    finalPath.clear();
}

std::vector<sf::Vector2i> AStar::getNeighbors(const Node& node) const {
    static const std::vector<sf::Vector2i> directions = {
        { 0, -1}, { 1, -1}, { 1,  0}, { 1,  1},
        { 0,  1}, {-1,  1}, {-1,  0}, {-1, -1}
    };

    std::vector<sf::Vector2i> neighbors;
    for (const auto& dir : directions) {
        int nx = node.x + dir.x;
        int ny = node.y + dir.y;
        if (isValidCell(nx, ny)) {
            neighbors.emplace_back(nx, ny);
        }
    }
    return neighbors;
}

void AStar::start() {
    if (startIndex.x == -1 || goalIndex.x == -1) {
        std::cerr << "Start or Goal not set properly.\n";
        finished = true;
        return;
    }

    // Inicializar estructuras para nueva búsqueda
    stateMap.assign(height, std::vector<State>(width, State::cEmpty));
    closedList.assign(height, std::vector<bool>(width, false));
    parentMap.assign(height, std::vector<sf::Vector2i>(width, {-1, -1}));
    gScore.assign(height, std::vector<float>(width, std::numeric_limits<float>::infinity()));
    openList = {};

    stateMap[startIndex.y][startIndex.x] = State::cStart;
    stateMap[goalIndex.y][goalIndex.x] = State::cGoal;

    // Nodo inicial
    Node startNode(startIndex.x, startIndex.y);
    startNode.gCost = 0;
    startNode.hCost = octileHeuristic(startIndex.x, startIndex.y, goalIndex.x, goalIndex.y);

    openList.push(startNode);
    gScore[startIndex.y][startIndex.x] = 0.0f;

    pathFound = false;
    finished = false;
}

bool AStar::step() {
    if (finished) {
        std::cout << "[DEBUG] finished == true, saliendo de step().\n";
        return false;
    }

    if (openList.empty()) {
        finished = true;
        pathFound = false;
        std::cout << "[DEBUG] openList vacío, no se encontró camino. finished = true.\n";
        return false;
    }

    currentNode = openList.top();
    openList.pop();

    std::cout << "[DEBUG] Procesando nodo (" << currentNode.x << ", " << currentNode.y << ") con gCost: " 
              << currentNode.gCost << ", hCost: " << currentNode.hCost << ", fCost: " << currentNode.fCost() << "\n";

    if (closedList[currentNode.y][currentNode.x]) {
        std::cout << "[DEBUG] Nodo ya cerrado (" << currentNode.x << ", " << currentNode.y << "), saltando.\n";
        return true;
    }

    closedList[currentNode.y][currentNode.x] = true;
    stateMap[currentNode.y][currentNode.x] = State::cClose;

    // Llegamos al objetivo
    if (currentNode.x == goalIndex.x && currentNode.y == goalIndex.y) {
        finished = true;
        pathFound = true;
        goalNode = currentNode;

        std::cout << "[DEBUG] Nodo objetivo alcanzado (" << currentNode.x << ", " << currentNode.y << ").\n";

        // Marcar camino en el mapa
        for (auto& pos : reconstructPath(goalNode)) {
            if (pos != startIndex && pos != goalIndex) {
                stateMap[pos.y][pos.x] = State::cPath;
                std::cout << "[DEBUG] Marcando camino en (" << pos.x << ", " << pos.y << ").\n";
            }
        }
        finalPath = reconstructPath(goalNode);
        std::cout << "Path to goal reached\n";
        return false;
    }

    // Procesar vecinos
    for (const auto& neighborPos : getNeighbors(currentNode)) {
        int nx = neighborPos.x;
        int ny = neighborPos.y;

        int occupancy = map.isOccupied(nx, ny);

        std::cout << "[DEBUG] Vecino (" << nx << ", " << ny << "), ocupación: " << occupancy 
                  << ", cerrado: " << closedList[ny][nx] << ", gScore actual: " << gScore[ny][nx] << "\n";

        // Permitir pasar por 's' y 'g', solo descartar si está ocupado (==1) o ya cerrado
        if (closedList[ny][nx] || occupancy == 1) {
            std::cout << "[DEBUG] Vecino (" << nx << ", " << ny << ") descartado.\n";
            continue;
        }

        float tentativeG = currentNode.gCost + octileHeuristic(currentNode.x, currentNode.y, nx, ny);

        std::cout << "[DEBUG] tentativeG para (" << nx << ", " << ny << "): " << tentativeG << "\n";

        if (tentativeG < gScore[ny][nx]) {
            std::cout << "[DEBUG] Actualizando gScore y padre para (" << nx << ", " << ny << ").\n";
            gScore[ny][nx] = tentativeG;
            parentMap[ny][nx] = {currentNode.x, currentNode.y};

            Node neighbor(nx, ny);
            neighbor.gCost = tentativeG;
            neighbor.hCost = octileHeuristic(nx, ny, goalIndex.x, goalIndex.y);

            openList.push(neighbor);
            std::cout << "[DEBUG] Nodo (" << nx << ", " << ny << ") añadido a openList con gCost: " 
                      << neighbor.gCost << ", hCost: " << neighbor.hCost << "\n";
        }
    }

    return true;
}



std::vector<sf::Vector2i> AStar::findPath() {
    start();
    while (!finished) {
        if (!step()) break;
    }
    return pathFound ? reconstructPath(goalNode) : std::vector<sf::Vector2i>{};
}

void AStar::updatemap() {
    startIndex = {-1, -1};
    goalIndex = {-1, -1};

    const auto& grid = map.getGrid();
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int cell = grid[y][x];
            if (cell == 's') {
                startIndex = {x, y};
                std::cout << "Updated startIndex: " << x << ", " << y << "\n";
            } else if (cell == 'g') {
                goalIndex = {x, y};
                std::cout << "Updated goalIndex: " << x << ", " << y << "\n";
            }
        }
    }
}

std::vector<sf::Vector2i> AStar::reconstructPath(const Node& endNode) const {
    std::vector<sf::Vector2i> path;
    sf::Vector2i current = {endNode.x, endNode.y};

    while (current != sf::Vector2i(-1, -1)) {
        path.push_back(current);
        current = parentMap[current.y][current.x];
    }

    std::reverse(path.begin(), path.end());
    return path;
}

void AStar::draw(sf::RenderTarget& target, float pixelsPerMeter, float metersPerCell) const {
    float cellSize = metersPerCell * pixelsPerMeter;
    float offsetX = -(width / 2.f) * cellSize;
    float offsetY = -(height / 2.f) * cellSize;

    sf::RectangleShape cellShape(sf::Vector2f(cellSize, cellSize));
    cellShape.setOutlineThickness(0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            switch (stateMap[y][x]) {
                case State::cStart:
                    cellShape.setFillColor(sf::Color::Red); break;
                case State::cEmpty:
                    continue;
                case State::cObstacle:
                    cellShape.setFillColor(sf::Color::Black); break;
                case State::cClose:
                    cellShape.setFillColor(sf::Color(100, 149, 237, 180)); break;
                case State::cPath:
                    cellShape.setFillColor(sf::Color::Yellow); break;
                case State::cGoal:
                    cellShape.setFillColor(sf::Color::Green); break;
                default:
                    continue;
            }

            cellShape.setPosition(offsetX + x * cellSize, offsetY + y * cellSize);
            target.draw(cellShape);
        }
    }
}

void AStar::drawFoundPath(sf::RenderTarget &target, float pixelsPerMeter, float metersPerCell) const
{
    if (finalPath.empty()) return; // No hay camino para dibujar

    float cellSize = metersPerCell * pixelsPerMeter;
    float offsetX = -(width / 2.f) * cellSize;
    float offsetY = -(height / 2.f) * cellSize;

    sf::RectangleShape cellShape(sf::Vector2f(cellSize, cellSize));
    cellShape.setFillColor(sf::Color::Yellow); // Color para el camino

    for (const auto& pos : finalPath) {
        // No dibujar start ni goal
        if (pos == startIndex || pos == goalIndex) continue;

        float x = offsetX + pos.x * cellSize;
        float y = offsetY + pos.y * cellSize;
        cellShape.setPosition(x, y);

        target.draw(cellShape);
    }
}

