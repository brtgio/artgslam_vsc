#pragma once
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

#include <vector>
#include <math.h>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/ViewController.hpp"

/**
 * @class LiveMap
 * @brief This class manages live map creation in "live mode", 
 * by dynamically updating the map from incoming ROS data (e.g., from RosHandler).
 * It builds an occupancy grid in real time and draws it using a ViewController.
 */
class LiveMap {
public:
    /**
     * @brief Constructor: initializes map size, resolution and controller reference
     * @param size Number of grid cells per side
     * @param resolution Size of each grid cell in meters
     * @param controller Reference to the view controller
     */
    LiveMap(int size, double resolution, ViewController& controller);

    /**
     * @brief Adds a single point (in real-world coordinates) to the internal point buffer
     * @param x Real-world X coordinate
     * @param y Real-world Y coordinate
     */
    void addPoint(double x, double y);

    /**
     * @brief Replaces current point buffer with a new set of coordinates
     * @param newX New set of X coordinates
     * @param newY New set of Y coordinates
     */
    void setPoints(const std::vector<double>& newX, const std::vector<double>& newY);

    /**
     * @brief Clears all stored real-world points
     */
    void clearPoints();

    /**
     * @brief Clears the occupancy grid (sets all cells to 0)
     */
    void clearGrid();
    
    /**
     * @brief Converts real-world coordinates to grid indices
     * @param x Vector of real-world X coordinates
     * @param y Vector of real-world Y coordinates
     * @param xGrid Output vector for grid X indices
     * @param yGrid Output vector for grid Y indices
     */
    void xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                 std::vector<int>& xGrid, std::vector<int>& yGrid);

    /**
     * @brief Fills the grid using the given grid indices, marking cells as occupied (1)
     * @param xGrid Vector of X grid indices
     * @param yGrid Vector of Y grid indices
     */
    void fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid);

    /**
     * @brief Returns a constant reference to the full occupancy grid
     * @return Constant reference to grid
     */
    const std::vector<std::vector<int>>& getGrid() const { return grid; };

    /**
     * @brief Updates the grid from the internal point buffer
     */
    void updateGridFromPoints();

    /**
     * @brief Draws the live grid using the ViewController
     * @param target SFML render target
     */
    void drawLiveMap(sf::RenderTarget& target) const;

    /**
     * @brief Activates or deactivates live mode (when true, dynamic updates occur)
     * @param isActive True to activate, false to deactivate
     */
    void setActive(bool isActive) { isLivemodeActive = true; };

    /**
     * @brief Returns whether live mode is currently active
     * @return True if live mode is active
     */
    bool getIsActive() const { return isLivemodeActive; };

    /**
     * @brief Sets a special start cell in the grid (value = 's')
     * @param i Grid row index
     * @param j Grid column index
     */
    void setStart(int i, int j);

    /**
     * @brief Sets a special goal cell in the grid (value = 'g')
     * @param i Grid row index
     * @param j Grid column index
     */
    void setGoal(int i, int j);

private:
    ViewController& controller;           ///< Reference to the view controller for rendering
    int gridSize;                         ///< Map size (number of cells per side)
    double gridResolution;                ///< Size of each cell in real-world units

    std::vector<double> posX, posY;       ///< Buffer of real-world x/y points
    std::vector<std::vector<int>> grid;   ///< 2D occupancy grid (0 = free, 1 = occupied, 's' = start, 'g' = goal)

    bool originSet = false;               ///< Flag to know if the origin is initialized
    double originX = 0.0, originY = 0.0;  ///< Optional offset for positioning

    bool isLivemodeActive = false;        ///< Whether live updates are happening
};
