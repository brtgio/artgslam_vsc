#include "artgslam_vsc/RosHandler.hpp"

/**
 * @brief Constructs a RosHandler object.
 * 
 * Initializes ROS parameters for joystick axes and scales,
 * sets up publishers and subscribers for velocity commands,
 * joystick inputs, sonar data, and RAM usage monitoring.
 * Also initializes timers for periodic memory usage publishing.
 */
RosHandler::RosHandler()
: linear(1), angular(0), l_scale(0.5), a_scale(0.5)
{
    // Read joystick axis parameters from ROS parameter server or use defaults
    nh.param("axis_linear", linear, linear);
    nh.param("axis_angular", angular, angular);
    nh.param("scale_angular", a_scale, a_scale);
    nh.param("scale_linear", l_scale, l_scale);

    // Publisher for velocity commands to the wheeled mobile robot using ROSARIA
    // Tested with Adept Mobile Robots Amigobot
    vel_pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

    // Subscribe to joystick inputs
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &RosHandler::joyCallback, this);

    // Subscribe to sonar data published on the ROS topic "sonarFilterdata_bag"
    sonar_sub = nh.subscribe("sonarFilterdata_bag", 1000, &RosHandler::sonarPointReceiver, this);

    // Initialize last joystick input time for synchronizing robot movement (testing pending)
    last_joy_time = ros::Time::now();

    // Publisher for RAM usage data (in kilobytes) for monitoring memory usage
    ram_pub = nh.advertise<std_msgs::Int32>("ram_usage_kb", 10);

    // Timer to periodically publish RAM usage every 1 second
    ram_timer = nh.createTimer(ros::Duration(1.0), &RosHandler::publishMemoryUsage, this);
}

/**
 * @brief Gets the vector of sonar points received from the sensor.
 * 
 * @return const reference to a vector containing sonar points.
 */
const std::vector<geometry_msgs::Point32>& RosHandler::getSonarPoints() const
{
    return sonarPoints;
}

/**
 * @brief Callback function for joystick input messages.
 * 
 * Processes joystick inputs to compute linear and angular velocities,
 * publishes velocity commands to control the robot,
 * and logs the current velocity values.
 * 
 * @param joy Const pointer to the joystick message received.
 */
void RosHandler::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    // Calculate time elapsed since last joystick message for potential synchronization
    ros::Time now = ros::Time::now();
    ros::Duration delta = now - last_joy_time;
    last_joy_time = now;
    last_dt = delta.toSec();

    // Create velocity command based on joystick input and scale parameters
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale * joy->axes[angular];
    twist.linear.x = l_scale * joy->axes[linear];

    // Store current velocities for external access
    current_linear_velocity = twist.linear.x;
    current_angular_velocity = twist.angular.z;

    // Publish velocity command to control the robot
    vel_pub.publish(twist);

    // Log current velocities for debugging
    ROS_INFO_STREAM("[JOY] Linear Velocity: " << twist.linear.x 
                    << " | Angular Velocity: " << twist.angular.z);
}

/**
 * @brief Callback function to receive sonar points from ROS topic.
 * 
 * Adds received sonar point to internal buffer and logs the coordinates.
 * 
 * @param msg Const pointer to the sonar point message received.
 */
void RosHandler::sonarPointReceiver(const geometry_msgs::Point32::ConstPtr &msg)
{
    // Append new sonar point to vector
    sonarPoints.push_back(*msg);

    // Log received sonar point coordinates
    ROS_INFO_STREAM("[SONAR] Point received -> X: " << msg->x
                    << " | Y: " << msg->y
                    << " | Z: " << msg->z);
}

/**
 * @brief Reads the current process memory usage (Resident Set Size) in kilobytes.
 * 
 * Parses the "/proc/self/status" file to extract the VmRSS value.
 * 
 * @return Memory usage in kilobytes, or -1 if reading failed.
 */
inline long RosHandler::getMemoryUsageKB() {
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind("VmRSS:", 0) == 0) { // Line starts with "VmRSS:"
            long kb;
            sscanf(line.c_str(), "VmRSS: %ld kB", &kb);
            return kb;
        }
    }
    return -1; // Return -1 if memory info could not be read
}

/**
 * @brief Timer callback function to publish the current memory usage.
 * 
 * Publishes the memory usage (in KB) to a ROS topic for monitoring.
 * 
 * @param event ROS timer event information (unused).
 */
inline void RosHandler::publishMemoryUsage(const ros::TimerEvent&) {
    std_msgs::Int32 msg;
    msg.data = static_cast<int>(getMemoryUsageKB());
    ram_pub.publish(msg);
}
