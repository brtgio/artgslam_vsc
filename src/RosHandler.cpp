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
    sonar_sub = nh.subscribe("sonarRawdata_bag", 1000, &RosHandler::sonarPointReceiver, this);
}

// Getter para los puntos del sonar
const std::vector<geometry_msgs::Point32>& RosHandler::getSonarPoints() const
{
    return sonarPoints;
}

// Callback del joystick
void RosHandler::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
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
