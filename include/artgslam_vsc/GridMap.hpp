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
#include <cmath>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/ViewController.hpp"

/**
 * @brief Handles the creation and management of map data.
 * 
 * It stores (x, y) coordinates from sonar readings, converts them
 * into grid cell indices, and fills a 2D occupancy grid.
 */
class GridMap
{
private:
    int gridSize;                         ///< Number of cells in the grid (map size)
    double gridResolution;               ///< Size of each cell in meters
    std::vector<double> posX, posY;      ///< Real-world coordinates from sonar data
    std::vector<std::vector<int>> grid;  ///< 2D occupancy grid

    ViewController& controller;          ///< Reference to view controller (for proper rendering)

public:
    /**
     * @brief Constructor.
     * 
     * @param size Grid dimension (number of cells per side)
     * @param resolution Size of each cell in meters
     * @param controller Reference to the ViewController for rendering
     */
    GridMap(int size, double resolution, ViewController& controller);

    /**
     * @brief Getters for real-world x coordinates
     * @return Vector of x coordinates
     */
    const std::vector<double>& getRealX() const { return posX; }

    /**
     * @brief Getters for real-world y coordinates
     * @return Vector of y coordinates
     */
    const std::vector<double>& getRealY() const { return posY; }

    /**
     * @brief Convert a real-world x coordinate to grid index.
     * @param realX X coordinate in meters
     * @return Corresponding grid index
     */
    int getCellIndexX(double realX) const;

    /**
     * @brief Convert a real-world y coordinate to grid index.
     * @param realY Y coordinate in meters
     * @return Corresponding grid index
     */
    int getCellIndexY(double realY) const;

    /**
     * @brief Add a point to the grid.
     * 
     * @param x Real-world x coordinate
     * @param y Real-world y coordinate
     */
    void addPoints(double x, double y);

    /**
     * @brief Replace all current points with new ones.
     * 
     * @param newX New vector of x coordinates
     * @param newY New vector of y coordinates
     */
    void setPoints(const std::vector<double>& newX, const std::vector<double>& newY);

    /**
     * @brief Clear all stored real-world points.
     */
    void clearPoints();

    /**
     * @brief Set the start point in the grid (stored as ASCII 's').
     * 
     * @param i Row index
     * @param j Column index
     */
    void setStart(int i, int j);

    /**
     * @brief Set the goal point in the grid (stored as ASCII 'g').
     * 
     * @param i Row index
     * @param j Column index
     */
    void setGoal(int i, int j);

    /**
     * @brief Query cell status.
     * 
     * @param i Row index
     * @param j Column index
     * @return 1 = occupied, 0 = free, ASCII 's' = start, ASCII 'g' = goal
     */
    int isOccupied(int i, int j) const;

    /**
     * @brief Convert real-world coordinates to grid indices.
     * 
     * @param x Input vector of x coordinates
     * @param y Input vector of y coordinates
     * @param xGrid Output vector of x indices
     * @param yGrid Output vector of y indices
     */
    void xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                 std::vector<int>& xGrid, std::vector<int>& yGrid);

    /**
     * @brief Fill the occupancy grid using grid index vectors.
     * 
     * @param xGrid Vector of x indices
     * @param yGrid Vector of y indices
     */
    void fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid);

    /**
     * @brief Get a const reference to the occupancy grid.
     * @return 2D grid of occupancy values
     */
    const std::vector<std::vector<int>>& getGrid() const;

    /**
     * @brief Clear the entire occupancy grid.
     */
    void clearGridMap();

    /**
     * @brief Clear start or goal markers from a cell.
     * 
     * @param cellIndex Index of the cell to clear
     */
    void clearSetPoints(sf::Vector2i cellIndex);

    /**
     * @brief Draw the map using SFML. Only filled cells are rendered.
     * 
     * @param target Render target (usually the SFML window)
     * @param pixelsPerMeter Scale factor for drawing
     */
    void draw(sf::RenderTarget& target, float pixelsPerMeter) const;

    /**
     * @brief Get the number of cells per side.
     * 
     * @return Grid size (width or height)
     */
    int getMapSize() const { return gridSize; }
};
