#include <ros/ros.h>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/MapViewer.hpp"

/**
 * @brief Main entry point for the ARTG SLAM visualization node.
 * Initializes ROS and SFML window, then runs the visualization loop.
 */
int main(int argc, char** argv)
{
    // Initialize ROS node with a descriptive name
    ros::init(argc, argv, "artgslam_vsc_node");

    // Create an SFML window for rendering the visualizer
    sf::RenderWindow window(sf::VideoMode(1024, 768), "ARTG SLAM Visualizer");
    window.setFramerateLimit(60);  // Cap frame rate at 60 FPS for smooth rendering

    // Instantiate MapViewer, which handles:
    //  - Map rendering and overlays
    //  - Simulation and live data updates
    //  - ROS topic subscriptions and publishing
    //  - User input and camera control
    MapViewer mapViewer(window);

    // Main loop: run while ROS is active and window is open
    while (ros::ok() && mapViewer.isRunning())
    {
        // Process user events (keyboard, mouse, window events)
        mapViewer.processEvent();

        // Update application state (map updates, robot pose, live mode, etc.)
        mapViewer.update();

        // Render the current frame to the window
        mapViewer.render();

        // Handle incoming ROS messages and callbacks
        ros::spinOnce();
    }

    return 0;
}
