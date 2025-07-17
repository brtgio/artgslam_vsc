#include "artgslam_vsc/AStar.hpp"
#include "AStar.hpp"

AStar::AStar(GridMap &mapRef)
    : map(mapRef)
{
    width = map.getMapSize();
    height = width;

    const auto& grid = map.getGrid();
    stateMap.resize(height, std::vector<State>(width));

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int cell = grid[y][x];

            if (cell == 1) {
                stateMap[y][x] = State::cObstacle;
            } else if (cell == 's') {
                stateMap[y][x] = State::cStart;
                startIndex = {x, y};
            } else if (cell == 'g') {
                stateMap[y][x] = State::cGoal;
                goalIndex = {x, y};
            } else {
                stateMap[y][x] = State::cEmpty;
            }
        }
    }

    // Inicializamos la lista cerrada como falsa (sin explorar)
    closedList.resize(height, std::vector<bool>(width, false));
}

std::vector<sf::Vector2i> AStar::getNeighbors(const Node& node) const
{
    std::vector<sf::Vector2i> neighbors;

    // Direcciones de movimiento (8 vecinos, incluyendo diagonales)
    static const std::vector<sf::Vector2i> directions = {
        { 0, -1}, // arriba
        { 1, -1}, // arriba-derecha
        { 1,  0}, // derecha
        { 1,  1}, // abajo-derecha
        { 0,  1}, // abajo
        {-1,  1}, // abajo-izquierda
        {-1,  0}, // izquierda
        {-1, -1}  // arriba-izquierda
    };

    // Revisamos qué vecinos son válidos
    for (const auto& dir : directions) {
        int newX = node.x + dir.x;
        int newY = node.y + dir.y;

        if (isValidCell(newX, newY)) {
            neighbors.emplace_back(newX, newY);
        }
    }

    return neighbors;
}

std::vector<sf::Vector2i> AStar::findPath(sf::Vector2i start, sf::Vector2i goal)
{
    // Limpiamos el mapa y las listas de estado
    stateMap.assign(height, std::vector<State>(width, State::cEmpty));
    closedList.assign(height, std::vector<bool>(width, false));
    openList = {};

    stateMap[start.y][start.x] = State::cStart;
    stateMap[goal.y][goal.x] = State::cGoal;

    // Inicializamos el nodo inicial
    Node startNode(start.x, start.y);
    startNode.gCost = 0;
    startNode.hCost = octileHeuristic(start.x, start.y, goal.x, goal.y);
    openList.push(startNode);

    // Bucle principal del algoritmo
    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        // Si ya fue procesado, lo ignoramos
        if (closedList[current.y][current.x]) continue;

        // Marcamos el nodo como cerrado
        closedList[current.y][current.x] = true;
        stateMap[current.y][current.x] = State::cClose;

        // Si llegamos a la meta, reconstruimos el camino
        if (current.x == goal.x && current.y == goal.y) {
            return reconstructPath(current);
        }

        // Revisamos los vecinos del nodo actual
        for (const sf::Vector2i& dir : getNeighbors(current)) {
            int neighborX = dir.x;
            int neighborY = dir.y;

            // Saltamos vecinos inválidos, cerrados o con obstáculos
            if (!isValidCell(neighborX, neighborY) ||
                closedList[neighborY][neighborX] ||
                map.isOccupied(neighborX, neighborY)) {
                continue;
            }

            float gCost = current.gCost + octileHeuristic(current.x, current.y, neighborX, neighborY);
            float hCost = octileHeuristic(neighborX, neighborY, goal.x, goal.y);

            // Creamos el nodo vecino
            Node neighbor(neighborX, neighborY);
            neighbor.gCost = gCost;
            neighbor.hCost = hCost;
            neighbor.parentX = current.x;
            neighbor.parentY = current.y;
            neighbor.parent = sf::Vector2i(current.x, current.y);

            // Agregamos a la lista abierta
            openList.push(neighbor);
        }
    }

    // Si no hay camino, devolvemos lista vacía
    return {};
}


