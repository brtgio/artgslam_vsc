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


#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>         // For std::setprecision
#include "GridMap.hpp"
#include "LiveMap.hpp"
#include "ViewController.hpp"
#include "tinyfiledialogs.h"

/**
 * @brief Manages file input/output operations for grid map visualization.
 * 
 * This class handles loading data from files to populate the map,
 * saving coordinate data, and exporting screenshots.
 */
class FileManager
{
private:
    GridMap& map;                 ///< Reference to the grid map object
    LiveMap& lmap;
    std::string loadedFilename;  ///< Path of the last loaded or saved file

public:
    /**
     * @brief Constructor.
     * 
     * @param mapRef Reference to the GridMap to work with
     */
    FileManager(GridMap& mapRef,LiveMap& lmapRef);

    /**
     * @brief Opens a file dialog to select a file and loads its data into the map.
     */
    void loadDialog();

    /**
     * @brief Opens a file dialog to save the current data (coordinates).
     */
    void saveDialog();

    /**
     * @brief Loads coordinate data from a file into x and y vectors.
     * 
     * @param filename Path of the file to read from
     * @param x Output vector of x coordinates
     * @param y Output vector of y coordinates
     */
    void dataLoad(const std::string& filename, std::vector<double>& x, std::vector<double>& y);

    /**
     * @brief Saves x and y coordinate data to a file.
     * 
     * @param filename Path of the file to write to
     * @param x Vector of x coordinates
     * @param y Vector of y coordinates
     */
    void saveData(const std::string& filename, std::vector<double>& x, std::vector<double>& y);

    /**
     * @brief Saves a screenshot of the current visualization to an image file.
     * 
     * @param filename Path of the image file to create
     */
    void saveScreen(const std::string& filename);
};
