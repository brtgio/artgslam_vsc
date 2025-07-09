#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <vector>
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <iostream>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32.h>   // <--- para publicar uso de RAM
#include <fstream>            // <--- para leer /proc/self/status

class RosHandler{

    public:
        RosHandler();

        const std::vector<geometry_msgs::Point32>& getSonarPoints() const;
        double getLinearVelocity() const { return current_linear_velocity; }
        double getAngularVelocity() const { return current_angular_velocity; }
        float getlast_dt() const{return last_dt;};

    private:
        ros::NodeHandle nh;

        //variables use to publish or resive data
        int linear, angular;
        double l_scale, a_scale;
        std::vector<geometry_msgs::Point32> sonarPoints;
        double current_linear_velocity = 0.0;
        double current_angular_velocity = 0.0;
        float last_dt = 0.0;
        ros::Time last_joy_time;

        //subs and pubs 
        ros::Publisher vel_pub;
        ros::Subscriber sonar_sub;
        ros::Subscriber joy_sub;

        // NUEVO: Publisher para uso de RAM y timer
        ros::Publisher ram_pub;
        ros::Timer ram_timer;

        void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
        void sonarPointReceiver(const geometry_msgs::Point32::ConstPtr& sonar);

        // NUEVO: función para obtener uso de memoria en KB
        long getMemoryUsageKB();

        // NUEVO: función que publica RAM periódicamente
        void publishMemoryUsage(const ros::TimerEvent&);

};