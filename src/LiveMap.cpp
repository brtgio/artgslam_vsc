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

#include <artgslam_vsc/LiveMap.hpp>
#include <iostream> // For debugging/logging

/**
 * @brief Constructs a LiveMap object with specified grid size and resolution.
 * Initializes the occupancy grid and links to the ViewController.
 * @param size Number of grid cells per side (square grid).
 * @param resolution Cell size in meters.
 * @param controller Reference to ViewController for visualization parameters.
 */
LiveMap::LiveMap(int size, double resolution, ViewController& controller)
    : gridSize(size), gridResolution(resolution), controller(controller), originSet(false)
{
    // Initialize grid to all free cells (0)
    grid.resize(gridSize, std::vector<int>(gridSize, 0));

    // Clear stored real-world points
    posX.clear();
    posY.clear();
}

/**
 * @brief Adds a new real-world point to the list.
 * The first point added is used to set the origin for coordinate conversion.
 * @param x X coordinate in meters.
 * @param y Y coordinate in meters.
 */
void LiveMap::addPoint(double x, double y)
{
    if (!originSet) {
        originX = x;
        originY = y;
        originSet = true;
    }
    posX.push_back(x);
    posY.push_back(y);
}

/**
 * @brief Replaces stored points with a new set.
 * @param newX Vector of X coordinates.
 * @param newY Vector of Y coordinates.
 */
void LiveMap::setPoints(const std::vector<double>& newX, const std::vector<double>& newY)
{
    posX = newX;
    posY = newY;
}

/**
 * @brief Clears all stored real-world points and resets the origin flag.
 */
void LiveMap::clearPoints()
{
    posX.clear();
    posY.clear();
    originSet = false;
}

/**
 * @brief Resets the occupancy grid cells to free (0).
 */
void LiveMap::clearGrid()
{
    for (auto& row : grid) {
        std::fill(row.begin(), row.end(), 0);
    }
}

/**
 * @brief Converts real-world coordinates to grid indices relative to the origin.
 * Points outside the grid are ignored.
 * @param x Vector of X coordinates.
 * @param y Vector of Y coordinates.
 * @param xGrid Output vector of grid X indices.
 * @param yGrid Output vector of grid Y indices.
 */
void LiveMap::xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                      std::vector<int>& xGrid, std::vector<int>& yGrid)
{
    if (x.size() != y.size() || !originSet) return;

    xGrid.clear();
    yGrid.clear();

    int halfGrid = gridSize / 2;

    for (size_t i = 0; i < x.size(); ++i) {
        double shiftedX = x[i] - originX;
        double shiftedY = y[i] - originY;

        int xIdx = static_cast<int>(std::round(shiftedX / gridResolution)) + halfGrid;
        int yIdx = static_cast<int>(std::round(shiftedY / gridResolution)) + halfGrid;

        // Ignore points outside grid bounds
        if (xIdx < 0 || xIdx >= gridSize || yIdx < 0 || yIdx >= gridSize)
            continue;

        xGrid.push_back(xIdx);
        yGrid.push_back(yIdx);
    }
}

/**
 * @brief Fills the grid with obstacles at specified grid indices.
 * Previous occupancy data is cleared.
 * @param xGrid Vector of X grid indices.
 * @param yGrid Vector of Y grid indices.
 */
void LiveMap::fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid)
{
    if (xGrid.size() != yGrid.size()) return;

    clearGrid();

    for (size_t i = 0; i < xGrid.size(); ++i) {
        int xIdx = xGrid[i];
        int yIdx = yGrid[i];

        grid[yIdx][xIdx] = 1; // Mark cell as occupied
    }
}

/**
 * @brief Updates the occupancy grid based on the stored real-world points.
 */
void LiveMap::updateGridFromPoints()
{
    if (posX.empty() || posY.empty()) return;

    std::vector<int> xGrid, yGrid;
    xy2Grid(posX, posY, xGrid, yGrid);
    fillGrid(xGrid, yGrid);
}

/**
 * @brief Draws occupied cells of the live map on the given SFML render target.
 * The drawing is centered and scaled according to the controller parameters.
 * @param target SFML RenderTarget to draw on.
 */
void LiveMap::drawLiveMap(sf::RenderTarget& target) const
{
    if (grid.empty()) return;

    float metersPerCell = controller.getMetersPerCell();   // e.g., 0.1
    float pixelsPerMeter = controller.getPixelsPerMeter(); // e.g., 50.0

    float cellSize = metersPerCell * pixelsPerMeter;

    int rows = static_cast<int>(grid.size());
    int cols = static_cast<int>(grid[0].size());

    int halfCols = cols / 2;
    int halfRows = rows / 2;

    float zoom = controller.getZoom();

    sf::RectangleShape cellShape;
    cellShape.setFillColor(sf::Color::Magenta);
    cellShape.setSize(sf::Vector2f(cellSize / zoom, cellSize / zoom));

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (grid[row][col] == 1) {
                int cellX = col - halfCols;
                int cellY = row - halfRows;

                float x = static_cast<float>(cellX) * cellSize / zoom;
                float y = static_cast<float>(cellY) * cellSize / zoom;

                cellShape.setPosition(x, y);
                target.draw(cellShape);
            }
        }
    }
}

/**
 * @brief Marks a cell as the goal in the grid using ASCII code 'g' (103).
 * @param i Row index.
 * @param j Column index.
 */
void LiveMap::setGoal(int i, int j)
{
    grid[i][j] = 103; // ASCII 'g'
}

/**
 * @brief Marks a cell as the start in the grid using ASCII code 's' (115).
 * @param i Row index.
 * @param j Column index.
 */
void LiveMap::setStart(int i, int j)
{
    grid[i][j] = 115; // ASCII 's'
}
