#include "artgslam_vsc/RosHandler.hpp"

// Constructor
RosHandler::RosHandler()
: linear(1), angular(0), l_scale(0.5), a_scale(0.5)
{
    // Carga parámetros desde el servidor de parámetros o usa los valores por defecto
    nh.param("axis_linear", linear, linear);
    nh.param("axis_angular", angular, angular);
    nh.param("scale_angular", a_scale, a_scale);
    nh.param("scale_linear", l_scale, l_scale);

    // Publicador de velocidades
    vel_pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

    // Suscriptor al joystick
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &RosHandler::joyCallback, this);

    // Suscriptor al sonar
    sonar_sub = nh.subscribe("sonarFilterdata_bag", 1000, &RosHandler::sonarPointReceiver, this);
    last_joy_time = ros::Time::now();

    //Monitoreo de RAM
    ram_pub = nh.advertise<std_msgs::Int32>("ram_usage_kb", 10);
    ram_timer = nh.createTimer(ros::Duration(1.0), &RosHandler::publishMemoryUsage, this);

}

// Getter para los puntos del sonar
const std::vector<geometry_msgs::Point32>& RosHandler::getSonarPoints() const
{
    return sonarPoints;
}

// Callback del joystick
void RosHandler::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    ros::Time now = ros::Time::now();
    ros::Duration delta = now - last_joy_time;
    last_joy_time = now;
    last_dt = delta.toSec();
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale * joy->axes[angular];
    twist.linear.x = l_scale * joy->axes[linear];
    
    current_linear_velocity = twist.linear.x;
    current_angular_velocity = twist.angular.z;

    vel_pub.publish(twist);

    ROS_INFO_STREAM("[JOY] Velocidad Linear: " << twist.linear.x 
                    << " | Velocidad Angular: " << twist.angular.z);
}

// Callback del sonar
void RosHandler::sonarPointReceiver(const geometry_msgs::Point32::ConstPtr &msg)
{
    sonarPoints.push_back(*msg);

    ROS_INFO_STREAM("[SONAR] Punto recibido -> X: " << msg->x
                    << " | Y: " << msg->y
                    << " | Z: " << msg->z);
}

inline long RosHandler::getMemoryUsageKB() {
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            long kb;
            sscanf(line.c_str(), "VmRSS: %ld kB", &kb);
            return kb;
        }
    }
    return -1;
}

inline void RosHandler::publishMemoryUsage(const ros::TimerEvent&) {
    std_msgs::Int32 msg;
    msg.data = static_cast<int>(getMemoryUsageKB());
    ram_pub.publish(msg);
}
