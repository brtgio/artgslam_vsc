/* 
 * -----------------------------------------------------------------------------
 *  Copyright (c) 2025 Gilberto Ramos Valenzuela
 *
 *  This file is part of the Artgslam Visualizer project.
 *
 *  Licensed for personal, academic, and non-commercial use only.
 *  Commercial use of the complete application "Artgslam Visualizer" is prohibited
 *  without explicit permission from the copyright holder.
 *
 *  For full license details, see the LICENSE.txt file distributed with this software.
 * -----------------------------------------------------------------------------
 */

#include "artgslam_vsc/AStar.hpp"
#include <iostream>
#include <cmath>
#include <algorithm> // std::reverse
#include <limits>

/**
 * @brief Constructor initializes A* algorithm with a reference to the map.
 * It prepares all necessary data structures to manage the pathfinding.
 */
AStar::AStar(GridMap& mapRef)
    : map(mapRef) // Reference to the occupancy grid map
{
    width = map.getMapSize(); // Map width (assuming square map)
    height = width;

    // Initialize stateMap with all cells empty
    stateMap.resize(height, std::vector<State>(width, State::cEmpty));

    // Initialize closed list with false flags (unvisited)
    closedList.resize(height, std::vector<bool>(width, false));

    // Initialize parent map with invalid parent indices
    parentMap.resize(height, std::vector<sf::Vector2i>(width, {-1, -1}));

    // Initialize gScores to infinity (unexplored)
    gScore.resize(height, std::vector<float>(width, std::numeric_limits<float>::infinity()));

    // Initialize start and goal indices as invalid
    startIndex = {-1, -1};
    goalIndex = {-1, -1};

    pathFound = false; // No path found yet
    finished = false;  // Algorithm not finished yet
    finalPath.clear(); // Clear any previous path
}

/**
 * @brief Returns valid neighbors around a node in 8 directions.
 * @param node Current node.
 * @return Vector of neighbor cell coordinates.
 */
std::vector<sf::Vector2i> AStar::getNeighbors(const Node& node) const {
    // Directions: N, NE, E, SE, S, SW, W, NW
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

/**
 * @brief Initializes the pathfinding process.
 * Resets all data structures and prepares the start node.
 */
void AStar::start() {
    // Verify start and goal points are set
    if (startIndex.x == -1 || goalIndex.x == -1) {
        std::cerr << "Start or Goal not set properly.\n";
        finished = true;
        return;
    }

    // Reset all pathfinding structures
    stateMap.assign(height, std::vector<State>(width, State::cEmpty));
    closedList.assign(height, std::vector<bool>(width, false));
    parentMap.assign(height, std::vector<sf::Vector2i>(width, {-1, -1}));
    gScore.assign(height, std::vector<float>(width, std::numeric_limits<float>::infinity()));
    openList = {}; // Clear priority queue

    // Mark start and goal on stateMap
    stateMap[startIndex.y][startIndex.x] = State::cStart;
    stateMap[goalIndex.y][goalIndex.x] = State::cGoal;

    // Initialize the start node
    Node startNode(startIndex.x, startIndex.y);
    startNode.gCost = 0;
    startNode.hCost = octileHeuristic(startIndex.x, startIndex.y, goalIndex.x, goalIndex.y);

    openList.push(startNode);
    gScore[startIndex.y][startIndex.x] = 0.0f;

    pathFound = false;
    finished = false;
}

/**
 * @brief Performs one iteration (step) of the A* algorithm.
 * @return true if more steps are needed; false if finished.
 */
bool AStar::step() {
    if (finished) {
        std::cout << "[DEBUG] finished == true, exiting step().\n";
        return false;
    }

    if (openList.empty()) {
        // No nodes left to explore, path not found
        finished = true;
        pathFound = false;
        std::cout << "[DEBUG] openList empty, no path found.\n";
        return false;
    }

    currentNode = openList.top();
    openList.pop();

    std::cout << "[DEBUG] Processing node (" << currentNode.x << ", " << currentNode.y 
              << ") gCost: " << currentNode.gCost 
              << ", hCost: " << currentNode.hCost 
              << ", fCost: " << currentNode.fCost() << "\n";

    if (closedList[currentNode.y][currentNode.x]) {
        std::cout << "[DEBUG] Node already closed, skipping.\n";
        return true;
    }

    closedList[currentNode.y][currentNode.x] = true;
    stateMap[currentNode.y][currentNode.x] = State::cClose;

    if (currentNode.x == goalIndex.x && currentNode.y == goalIndex.y) {
        finished = true;
        pathFound = true;
        goalNode = currentNode;

        std::cout << "[DEBUG] Goal node reached.\n";

        // Mark the path cells (except start/goal)
        for (auto& pos : reconstructPath(goalNode)) {
            if (pos != startIndex && pos != goalIndex) {
                stateMap[pos.y][pos.x] = State::cPath;
                std::cout << "[DEBUG] Marking path cell (" << pos.x << ", " << pos.y << ").\n";
            }
        }

        finalPath = reconstructPath(goalNode);
        std::cout << "Path to goal reached\n";
        return false;
    }

    // Explore neighbors
    for (const auto& neighborPos : getNeighbors(currentNode)) {
        int nx = neighborPos.x;
        int ny = neighborPos.y;

        int occupancy = map.isOccupied(nx, ny);

        std::cout << "[DEBUG] Neighbor (" << nx << ", " << ny << ") occupancy: " << occupancy
                  << ", closed: " << closedList[ny][nx]
                  << ", current gScore: " << gScore[ny][nx] << "\n";

        if (closedList[ny][nx] || occupancy == 1) {
            std::cout << "[DEBUG] Neighbor discarded.\n";
            continue;
        }

        float tentativeG = currentNode.gCost + octileHeuristic(currentNode.x, currentNode.y, nx, ny);

        std::cout << "[DEBUG] tentativeG for (" << nx << ", " << ny << "): " << tentativeG << "\n";

        if (tentativeG < gScore[ny][nx]) {
            std::cout << "[DEBUG] Updating gScore and parent for (" << nx << ", " << ny << ").\n";
            gScore[ny][nx] = tentativeG;
            parentMap[ny][nx] = {currentNode.x, currentNode.y};

            Node neighbor(nx, ny);
            neighbor.gCost = tentativeG;
            neighbor.hCost = octileHeuristic(nx, ny, goalIndex.x, goalIndex.y);

            openList.push(neighbor);
            std::cout << "[DEBUG] Neighbor (" << nx << ", " << ny << ") added to openList.\n";
        }
    }

    return true;
}

/**
 * @brief Finds the complete path from start to goal.
 * @return Vector of cell positions along the path.
 */
std::vector<sf::Vector2i> AStar::findPath() {
    start();
    while (!finished) {
        if (!step()) break;
    }
    return pathFound ? reconstructPath(goalNode) : std::vector<sf::Vector2i>{};
}

/**
 * @brief Updates the start and goal positions by scanning the map for 's' and 'g' markers.
 */
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

/**
 * @brief Reconstructs the path from the goal node back to the start node.
 * @param endNode The goal node from which to trace back.
 * @return Vector of cell positions representing the path.
 */
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

/**
 * @brief Draws the current state of the map (open, closed, path, etc.) on the render target.
 * @param target SFML render target to draw on.
 * @param pixelsPerMeter Scaling factor for rendering.
 * @param metersPerCell Size of each grid cell in meters.
 */
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
                    cellShape.setFillColor(sf::Color::Red);
                    break;
                case State::cEmpty:
                    continue; // Skip empty cells
                case State::cObstacle:
                    cellShape.setFillColor(sf::Color::Black);
                    break;
                case State::cClose:
                    cellShape.setFillColor(sf::Color(100, 149, 237, 180)); // Light blue, semi-transparent
                    break;
                case State::cPath:
                    cellShape.setFillColor(sf::Color::Yellow);
                    break;
                case State::cGoal:
                    cellShape.setFillColor(sf::Color::Green);
                    break;
                default:
                    continue;
            }

            cellShape.setPosition(offsetX + x * cellSize, offsetY + y * cellSize);
            target.draw(cellShape);
        }
    }
}

/**
 * @brief Draws the final path found by the algorithm.
 * @param target SFML render target.
 * @param pixelsPerMeter Pixels per meter scale.
 * @param metersPerCell Size of grid cell in meters.
 */
void AStar::drawFoundPath(sf::RenderTarget &target, float pixelsPerMeter, float metersPerCell) const {
    if (finalPath.empty()) return; // No path to draw

    float cellSize = metersPerCell * pixelsPerMeter;
    float offsetX = -(width / 2.f) * cellSize;
    float offsetY = -(height / 2.f) * cellSize;

    sf::RectangleShape cellShape(sf::Vector2f(cellSize, cellSize));
    cellShape.setFillColor(sf::Color::Yellow);

    for (const auto& pos : finalPath) {
        if (pos == startIndex || pos == goalIndex) continue;

        float x = offsetX + pos.x * cellSize;
        float y = offsetY + pos.y * cellSize;
        cellShape.setPosition(x, y);

        target.draw(cellShape);
    }
}
