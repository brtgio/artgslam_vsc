#pragma once

#include "artgslam_vsc/MenuBar.hpp"
#include "artgslam_vsc/FileManager.hpp"
#include "artgslam_vsc/GridMap.hpp"
#include "artgslam_vsc/RosHandler.hpp"
#include "artgslam_vsc/ViewController.hpp"
#include "artgslam_vsc/RobotCreator.hpp"
#include "artgslam_vsc/UnicicleWmr.hpp"
#include "artgslam_vsc/LiveMap.hpp"
#include "artgslam_vsc/RightClickMapMenu.hpp"
#include "artgslam_vsc/AStar.hpp"
#include <SFML/Graphics.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>

/**
 * @class MapViewer
 * @brief This is the main class. It manages mouse/keyboard events and GUI integration.
 * It also coordinates the rendering and simulation components of the application.
 */
class MapViewer {
private:
    sf::RenderWindow& window;        ///< Reference to the SFML window for rendering

    sf::View view;                   ///< SFML camera view
    tgui::Gui gui;                   ///< GUI system for handling menus and widgets
    
    MenuBar menu;                   ///< Instance of the custom menu bar
    FileManager manager;           ///< Manages file loading/saving
    RosHandler roshandler;         ///< Handles communication with ROS (publishing/subscribing)
    ViewController controller;     ///< Manages zoom, panning, grid drawing, and coordinate conversions
    GridMap map;                   ///< Represents and stores the occupancy grid
    UnicicleWmr wmr;               ///< Simulated unicycle WMR (Wheeled Mobile Robot)
    LiveMap livemap;              ///< Handles live mapping mode based on ROS data
    RightClickMapMenu r_menu;     ///< Context menu for selecting start and goal positions
    AStar aStarsim;               ///< A* algorithm instance for path planning and animation

    bool running = true;               ///< Indicates whether the main loop is running
    sf::Vector2f worldXY;              ///< World coordinates (floating point)
    sf::Vector2i gridIndex;            ///< Current hovered grid cell
    sf::Vector2i gridIndex2copy;       ///< Temporary copy of a selected grid index

    bool astarAnimating = false;       ///< Indicates whether the A* animation is currently running

public:
    /**
     * @brief Constructor
     * @param win Reference to the SFML render window
     */
    MapViewer(sf::RenderWindow& win);

    /**
     * @brief Manages logic updates and user interaction
     */
    void update();

    /**
     * @brief Handles input events (mouse and keyboard)
     */
    void processEvent();

    /**
     * @brief Renders all visual components to the screen
     */
    void render();

    /**
     * @brief Returns whether the viewer should keep running
     * @return True if running, false otherwise
     */
    [[nodiscard]] bool isRunning() const;
};
