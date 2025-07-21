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

/* 
 * Main class that handles rendering, input processing, UI menus, 
 * and their associated actions for the SLAM or navigation viewer.
 */
class MapViewer {
private:
    sf::RenderWindow& window;        // Reference to the main SFML window used for rendering
    
    sf::View view;                   // 2D camera handler (zoom and pan functionality)
    tgui::Gui gui;                   // TGUI main object for managing GUI widgets

    MenuBar menu;                    // Down menu bar (File, View, etc.)
    FileManager manager;             // Handles saving/loading data to/from files
    RosHandler roshandler;           // Handles ROS communication and callbacks
    ViewController controller;       // Controls the view (camera movement and zoom)
    GridMap map;                     // Manages the internal 2D occupancy grid map
    UnicicleWmr wmr;                 // Simulates a unicycle-type wheeled mobile robot
    LiveMap livemap;                 // Handles live map updates in real time (from sensors or ROS)
    RightClickMapMenu r_menu;        // Manages context menu for right-click map interactions (e.g., set goal)
    AStar aStarsim;                  // A* path planning algorithm implementation

    bool running = true;             // Indicates if the main loop is running
    sf::Vector2f worldXY;            // Mouse position in world coordinates
    sf::Vector2i gridIndex;          // First selected grid index (e.g., start point for path planning)
    sf::Vector2i gridIndex2copy;     // Second selected grid index (e.g., goal point)

    bool astarAnimating = false;     // Flag indicating whether A* path animation is playing
    bool astarCompleted = false;     // Flag indicating whether A* path planning has finished

public:
    // Constructor (requires a reference to the render window)
    MapViewer(sf::RenderWindow& win);

    // Updates the simulation and internal logic
    void update();

    // Processes keyboard, mouse, and GUI events
    void processEvent();

    // Renders the GUI and simulation elements to the screen
    void render();

    // Checks if the viewer is still running
    [[nodiscard]] bool isRunning() const;
};
