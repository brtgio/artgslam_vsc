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


#include <SFML/Graphics.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/AllWidgets.hpp>
#include <ros/package.h>
#include <iostream>
#include <regex>
#include "artgslam_vsc/GridMap.hpp"
#include "artgslam_vsc/LiveMap.hpp"

/**
 * @class RightClickMapMenu
 * @brief Manages a contextual right-click menu displayed on top of a map.
 * Allows users to set the start and goal positions or clear the grid through TGUI buttons.
 */
class RightClickMapMenu {
public:
    /**
     * @brief Constructor: initializes the menu with references to the TGUI GUI system and map instances.
     * @param guiRef TGUI GUI object (must persist).
     * @param mapRef Reference to the static grid map.
     * @param livemapRef Reference to the live map (dynamic overlays, e.g., visited paths).
     */
    RightClickMapMenu(tgui::Gui& guiRef, GridMap &mapRef, LiveMap& livemapRef);

    /**
     * @brief Sets up the menu layout and widgets (start, goal, clear, etc.).
     */
    void setupWidgets();

    /**
     * @brief Displays the context menu at the given screen coordinates.
     * @param x X coordinate in world space.
     * @param y Y coordinate in world space.
     * @param gridIndex Cell index corresponding to the click.
     */
    void show(float x, float y, const sf::Vector2i& gridIndex);

    /**
     * @brief Hides the context menu.
     */
    void hide();

    /**
     * @brief Sets the visibility state of the menu.
     * @param show True to show the menu, false to hide.
     */
    void setVisible(bool show);

    /**
     * @brief Returns whether the menu is currently visible.
     * @return True if visible, false otherwise.
     */
    bool isVisible() const;

    /**
     * @brief Connects internal widget signals to their callbacks (e.g., button clicks).
     */
    void connectSignals();

    /**
     * @brief Checks if a world-space point is inside the menu bounds.
     * Useful to avoid menu clicks being interpreted as map clicks.
     * @param point World coordinate point.
     * @return True if point is inside the menu, false otherwise.
     */
    bool containsPoint(const sf::Vector2f& point) const;

    /**
     * @brief Returns true if both the start and goal have been set by the user.
     * Used to determine when path planning can begin.
     * @return True if start and goal are active.
     */
    bool isSet() const { return isGoalActive && isStartActive; }

private:
    tgui::Gui& gui;                ///< GUI reference (owned externally)
    GridMap& gridmap;              ///< Reference to the static grid map
    LiveMap& livemap;              ///< Reference to the live map

    tgui::Group::Ptr container;          ///< Main container for the popup menu
    tgui::ChildWindow::Ptr panel;        ///< Panel that acts as the context menu window

    tgui::Button::Ptr start;       ///< Sets the start position
    tgui::Button::Ptr goal;        ///< Sets the goal position
    tgui::Button::Ptr clear;       ///< Clears the map
    tgui::Button::Ptr clearView;   ///< Clears overlays (visited cells, path, etc.)

    bool visible = false;           ///< Visibility flag

    sf::Vector2f worldXY_copy;     ///< Click position in world coordinates
    sf::Vector2i gridIndex_copy;   ///< Corresponding cell index

    sf::Vector2i startIndex;       ///< Current start cell index
    sf::Vector2i goalIndex;        ///< Current goal cell index

    bool isGoalActive = false;     ///< Flag: goal has been set
    bool isStartActive = false;    ///< Flag: start has been set
};
