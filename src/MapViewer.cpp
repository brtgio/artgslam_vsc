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

#include "artgslam_vsc/MapViewer.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>

/**
 * @brief Constructs a MapViewer instance.
 * 
 * Initializes references to the SFML window, GUI, controller, map, menu, ROS handler,
 * robot model, live map, right-click menu, and A* simulation.
 * Sets up menu callbacks and connects right-click menu signals.
 * 
 * @param win Reference to the SFML RenderWindow where rendering occurs.
 */
MapViewer::MapViewer(sf::RenderWindow& win)
    : window(win)                           // Store reference to SFML window
    , view(window.getDefaultView())        // Initialize default camera view
    , gui(win)                            // Initialize GUI with window reference
    , controller(win, 0.1f, 50.0f, view)  // Initialize ViewController with parameters
    , map(1000, 0.1, controller)           // Initialize GridMap with size, resolution, controller
    , manager(map)                         // FileManager with reference to GridMap
    , menu(gui)                           // MenuBar with GUI reference
    , roshandler()                       // ROS data handler for sensors and velocity
    , wmr()                              // Wheeled Mobile Robot model
    , livemap(1000, 0.1, controller)      // LiveMap with same size and resolution
    , r_menu(gui, map, livemap)           // Right-click menu with GUI, map, and livemap refs
    , aStarsim(map)                       // A* pathfinding simulator with GridMap
{
    r_menu.connectSignals();  /**< Connect right-click menu event callbacks */

    // Set callback functions for menu bar actions
    menu.setCallbacks(
        [this]() { manager.loadDialog(); },              /**< Load map file dialog */
        [this]() { manager.saveDialog(); },              /**< Save map file dialog */
        [this]() { /* TODO: Implement image saving functionality */ },
        [this]() { running = false; window.close(); },   /**< Exit application */
        [this]() { controller.reset(); },                /**< Reset camera view */
        [this]() { map.clearGridMap(); },                /**< Clear the grid map */
        [this]() {                                        /**< Open robot creator tool */
            RobotCreator creator(wmr);
            creator.run();
        },
        [this]() {                                        /**< Run A* pathfinding animation */
            aStarsim.updatemap();  /**< Update start and goal points */
            aStarsim.start();      /**< Start A* animation */
            astarAnimating = true;
        }
    );
}

/**
 * @brief Updates application logic per frame.
 * 
 * Updates mouse position and grid cell info, live mode sensor data, A* animation,
 * and robot model position.
 */
void MapViewer::update()
{
    int gridSize = map.getMapSize();

    // Get the grid cell currently under the mouse cursor
    sf::Vector2i gridIndex = controller.getHoveredCell(gridSize);

    // Get mouse position in world coordinates (meters)
    sf::Vector2f worldPos = controller.getMouseWorldPosition();

    // Prepare coordinate status string with fixed precision
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "Mouse: (" << worldPos.x << ", " << worldPos.y << ") m";

    if (gridIndex.x != -1 && gridIndex.y != -1) {
        oss << " | Grid: (" << gridIndex.x << ", " << gridIndex.y << ")";
    } else {
        oss << " | Out of bounds";
    }

    // Update coordinate display in status bar
    menu.updateCoordinates(oss.str());

    // -------- Live Mode: real-time sensor updates --------
    if (menu.getLiveMode()) {
        livemap.clearPoints();  /**< Clear existing sonar points */
        livemap.clearGrid();    /**< Clear live occupancy grid */

        // Update robot velocity from ROS data
        double v = roshandler.getLinearVelocity();
        double w = roshandler.getAngularVelocity();
        wmr.setVelocity(v, w);

        // Add sonar points to live map
        const auto& sonar = roshandler.getSonarPoints();
        for (const auto& p : sonar) {
            livemap.addPoint(p.x, p.y);
        }

        // Update live map grid with sonar points
        livemap.updateGridFromPoints();
    }

    // -------- A* Animation: step-by-step simulation --------
    if (astarAnimating) {
        if (!aStarsim.isFinished()) {
            bool continueAnim = aStarsim.step();
            if (!continueAnim) {
                astarAnimating = false;
            }
        } else {
            astarAnimating = false;
        }
    }

    // Update robot model position using ROS delta time
    wmr.update(roshandler.getLastDeltaTime());
}

/**
 * @brief Processes SFML window events.
 * 
 * Handles user inputs including window closing, mouse clicks, and GUI events.
 * Shows or hides the right-click menu on mouse events.
 */
void MapViewer::processEvent()
{
    sf::Event event;
    while (window.pollEvent(event)) {
        gui.handleEvent(event);        /**< Forward event to GUI system */
        controller.handleEvent(event); /**< Forward event to view controller */

        if (event.type == sf::Event::Closed) {
            running = false;
            window.close();            /**< Close window and exit */
        }

        if (event.type == sf::Event::MouseButtonPressed) {
            const int gridSize = map.getMapSize();
            const sf::Vector2i cell = controller.getHoveredCell(gridSize);
            const sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
            const sf::Vector2f pixelPosF(pixelPos);

            if (event.mouseButton.button == sf::Mouse::Right) {
                // Show context menu at mouse position for right-click
                r_menu.show(static_cast<float>(pixelPos.x),
                            static_cast<float>(pixelPos.y),
                            cell);
                r_menu.setVisible(true);
            } else if (event.mouseButton.button == sf::Mouse::Left) {
                // Hide menu if click is outside of the menu area
                if (r_menu.isVisible() && !r_menu.containsPoint(pixelPosF)) {
                    r_menu.setVisible(false);
                }
            }
        }
    }
}

/**
 * @brief Renders the entire map viewer frame.
 * 
 * Clears the window, draws the grid and axes, live or stored maps,
 * A* simulation, robot, GUI, and presents the final image.
 */
void MapViewer::render()
{
    window.clear(sf::Color::Black);       /**< Clear window with black background */
    controller.applyView();                /**< Apply current camera transform (zoom, pan) */

    controller.drawGrid(window);           /**< Draw grid lines */
    controller.drawAxes(window);           /**< Draw X and Y axes */

    // Draw live map or stored grid map based on live mode status
    if (menu.getLiveMode()) {
        const auto& gridLive = livemap.getGrid();
        if (gridLive.empty() || gridLive[0].empty()) {
            std::cout << "Live grid is empty!" << std::endl;
            window.setView(window.getDefaultView());
            gui.draw();
            window.display();
            return;
        }
        livemap.drawLiveMap(window);       /**< Draw live sensor data map */
    } else {
        const auto& gridMap = map.getGrid();
        if (gridMap.empty() || gridMap[0].empty()) {
            std::cout << "Grid map is empty!" << std::endl;
            window.setView(window.getDefaultView());
            gui.draw();
            window.display();
            return;
        }
        map.draw(window, controller.getPixelsPerMeter()); /**< Draw stored occupancy grid */
    }

    // Draw A* algorithm visualization while animating
    if (astarAnimating) {
        aStarsim.draw(window, 50.0f, controller.getMetersPerCell());
    }

    // Draw the final path found by A* (if any)
    aStarsim.drawFoundPath(window, 50.0f, controller.getMetersPerCell());

    // Draw robot’s current pose and orientation
    wmr.draw(window);

    // Draw GUI elements on top (using default view)
    window.setView(window.getDefaultView());
    gui.draw();

    // Display everything on screen
    window.display();
}

/**
 * @brief Checks if the MapViewer application is running.
 * 
 * @return true if the application is running and window is open, false otherwise.
 */
bool MapViewer::isRunning() const
{
    return running && window.isOpen();
}
