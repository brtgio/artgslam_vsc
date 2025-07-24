#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <cmath>

/**
 * @class RosHandler
 * @brief Handles ROS communication:
 *  - Publishes velocity commands
 *  - Subscribes to joystick and sonar data
 *  - Provides access to velocities and sonar points
 */
class RosHandler {
public:
    /// Constructor
    RosHandler();

    /// @return Sonar points received from ROS used to build the map
    const std::vector<geometry_msgs::Point32>& getSonarPoints() const;

    /// @return Current robot linear velocity for animation purposes
    double getLinearVelocity() const { return current_linear_velocity; }

    /// @return Current robot angular velocity for animation purposes
    double getAngularVelocity() const { return current_angular_velocity; }

    /// @return Time since last joystick update
    float getLastDeltaTime() const { return last_dt; }

private:
    ros::NodeHandle nh; ///< ROS node handle

    // Velocity scaling factors
    int linear, angular; ///< Raw joystick axis indices
    double l_scale, a_scale; ///< Linear and angular velocity scaling factors

    // Sensor data
    std::vector<geometry_msgs::Point32> sonarPoints; ///< Points from sonar sensor
    double current_linear_velocity = 0.0; ///< Current linear velocity
    double current_angular_velocity = 0.0; ///< Current angular velocity
    float last_dt = 0.0; ///< Time since last joystick message
    ros::Time last_joy_time; ///< Timestamp of last joystick message received

    // ROS publishers and subscribers
    ros::Publisher vel_pub; ///< Publisher for velocity commands
    ros::Subscriber sonar_sub; ///< Subscriber for sonar point messages
    ros::Subscriber joy_sub; ///< Subscriber for joystick messages

    // RAM monitoring
    ros::Publisher ram_pub; ///< Publisher for RAM usage info
    ros::Timer ram_timer; ///< Timer to trigger RAM publishing

    // Callbacks
    /**
     * @brief Callback for joystick messages.
     * Reads joystick values and publishes velocity commands.
     * @param joy Incoming joystick message
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    /**
     * @brief Callback for sonar point messages.
     * Receives sonar point data to be used in mapping.
     * @param sonar Incoming sonar point message
     */
    void sonarPointReceiver(const geometry_msgs::Point32::ConstPtr& sonar);

    // RAM usage utilities
    /**
     * @brief Reads current RAM usage of the process in KB.
     * @return RAM usage in kilobytes
     */
    long getMemoryUsageKB();

    /**
     * @brief Periodically publishes the RAM usage to a ROS topic.
     * @param event Timer event info
     */
    void publishMemoryUsage(const ros::TimerEvent&);
};
