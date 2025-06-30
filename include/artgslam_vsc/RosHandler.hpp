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

class RosHandler{

    public:
        RosHandler();
         const std::vector<geometry_msgs::Point32>& getSonarPoints() const;


    private:
        ros::NodeHandle nh;

        //variables use to publish or resive data
        int linear, angular;
        double l_scale, a_scale;
        std::vector<geometry_msgs::Point32> sonarPoints;
        

        //subs and pubs 
        ros::Publisher vel_pub;
        ros::Subscriber sonar_sub;
        ros::Subscriber joy_sub;
        

        void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
        void sonarPointReceiver(const geometry_msgs::Point32::ConstPtr& sonar);


      

};