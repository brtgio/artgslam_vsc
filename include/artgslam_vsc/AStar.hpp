#pragma once

#include <cmath>
#include <queue>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/GridMap.hpp"
#include "artgslam_vsc/Node.hpp"

/// \class AStar
/// \brief This class handles the A* algorithm for path planning. 
/// It receives start and goal coordinates from the GridMap,
/// performs the algorithm step-by-step, and supports animation and path retrieval.
class AStar {
public:
    /// \enum State
    /// \brief Enum representing the visual state of each grid cell
    enum class State {
        cStart,     ///< Start cell
        cEmpty,     ///< Unvisited cell
        cObstacle,  ///< Obstacle cell
        cClose,     ///< Closed/visited cell
        cPath,      ///< Final path cell
        cGoal       ///< Goal cell
    };

    /// \brief Constructor with GridMap reference
    /// \param mapRef Reference to the occupancy grid map
    AStar(GridMap& mapRef);

    // --- Algorithm control functions ---

    /// \brief Initializes the A* algorithm.
    /// Prepares internal structures and sets start/goal positions.
    void start(); 

    /// \brief Performs one step of the A* algorithm.
    /// \return true if a step was taken, false if algorithm is finished.
    bool step();  

    /// \brief Executes the full A* algorithm from start to goal in one call.
    /// \return Vector of grid coordinates representing the path.
    std::vector<sf::Vector2i> findPath();

    /// \brief Updates the internal state map based on the current GridMap.
    /// Should be called if the GridMap is modified (e.g., new obstacles).
    void updatemap();

    // --- Visualization methods ---

    /// \brief Draws the current algorithm state to the given SFML render target.
    /// Can be used for animated A* visualizations.
    void draw(sf::RenderTarget& target, float pixelsPerMeter, float metersPerCell) const;

    /// \brief Draws the final path, if found.
    void drawFoundPath(sf::RenderTarget& target, float pixelsPerMeter, float metersPerCell) const;

    /// \brief Returns whether the algorithm has finished executing.
    bool isFinished() const { return finished; }

    /// \brief Returns whether a path was found from start to goal.
    bool isPathFound() const { return pathFound; }

    /// \brief Returns the current node being evaluated (useful for debugging or animation).
    const Node& getCurrentNode() const { return currentNode; }

    /// \brief Returns the current internal state map used for visualization.
    const std::vector<std::vector<State>>& getStateMap() const { return stateMap; }

private:
    GridMap& map;  ///< Reference to the associated occupancy grid map

    // --- Internal state variables ---

    int width = 0;                     ///< Grid width
    int height = 0;                    ///< Grid height
    bool pathFound = false;           ///< True if a valid path was found
    bool finished = false;            ///< True if the algorithm has completed

    sf::Vector2i startIndex = {-1, -1};  ///< Start position (grid indices)
    sf::Vector2i goalIndex  = {-1, -1};  ///< Goal position (grid indices)
    Node currentNode;                   ///< Node currently being processed
    Node goalNode;                      ///< Target goal node

    // --- A* algorithm data structures ---
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList; ///< Priority queue for open nodes
    std::vector<std::vector<bool>> closedList;        ///< Tracks visited nodes
    std::vector<std::vector<float>> gScore;           ///< Cost from start to each cell
    std::vector<std::vector<sf::Vector2i>> parentMap; ///< For path reconstruction
    std::vector<std::vector<State>> stateMap;         ///< State of each cell for visualization
    std::vector<sf::Vector2i> finalPath;              ///< Final path from start to goal

    // --- Heuristic functions ---

    /// \brief Euclidean distance heuristic.
    double euclideanHeuristic(int x1, int y1, int x2, int y2);

    /// \brief Octile distance heuristic (better suited for 8-connected grids).
    double octileHeuristic(int x1, int y1, int x2, int y2) {
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
    }

    /// \brief Checks if the given cell coordinates are within the grid bounds.
    bool isValidCell(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    /// \brief Returns the valid neighbor cells of a given node.
    std::vector<sf::Vector2i> getNeighbors(const Node& node) const;

    /// \brief Reconstructs the final path from the goal node by tracing the parent map.
    std::vector<sf::Vector2i> reconstructPath(const Node& endNode) const;
};
