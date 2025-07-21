#include <ros/ros.h>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/MapViewer.hpp"

/* This is a grid map viewer for navigation or SLAM applications. 
   It supports live map creation via ROS 1. */

/* This is the main loop. It handles the main events. */
int main(int argc, char** argv)
{
    // Initialize ROS 
    ros::init(argc, argv, "artgslam_vsc_node");

    // Create a window with the desired screen resolution and give a title to the window 
    sf::RenderWindow window(sf::VideoMode(1024, 768), "ARTG SLAM Visualizer");
    window.setFramerateLimit(60);  // Limit window FPS to 60

    // Initialize the MapViewer object
    MapViewer mapViewer(window);

    // Main loop
    while (ros::ok() && mapViewer.isRunning())
    {
        // Process all mouse and keyboard inputs
        mapViewer.processEvent();

        // Update the internal logic
        mapViewer.update();

        // Draw things on the screen
        mapViewer.render();

        // Process ROS callbacks 
        ros::spinOnce();
    }

    return 0;
}

