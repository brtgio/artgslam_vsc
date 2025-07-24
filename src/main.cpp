#include <ros/ros.h>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/MapViewer.hpp"

/**
 * @brief Main entry point for the ARTG SLAM visualization node.
 * 
 * Initializes the ROS node and SFML rendering window, then
 * runs the main visualization loop that processes events,
 * updates logic, renders visuals, and handles ROS callbacks.
 * 
 * @param argc Argument count from command line.
 * @param argv Argument vector from command line.
 * @return int Exit status code.
 */
int main(int argc, char** argv)
{
    // Initialize ROS node with a descriptive name
    ros::init(argc, argv, "artgslam_vsc_node");

    // Create an SFML window for rendering the visualizer
    sf::RenderWindow window(sf::VideoMode(1024, 768), "ARTG SLAM Visualizer");
    window.setFramerateLimit(60);  // Cap frame rate at 60 FPS for smooth rendering

    // Instantiate MapViewer, which manages rendering, input, and ROS integration
    MapViewer mapViewer(window);

    // Main loop: runs while ROS is active and window remains open
    while (ros::ok() && mapViewer.isRunning())
    {
        mapViewer.processEvent();  // Handle input events
        mapViewer.update();        // Update simulation and logic
        mapViewer.render();        // Render visualization
        ros::spinOnce();           // Process ROS callbacks
    }

    return 0;
}
