#ifndef WHEEL_ODOM_NODE_HPP
#define WHEEL_ODOM_NODE_HPP

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <math.h>

#include <iostream>


class WheelOdomNode : public ros::NodeHandle
{
    
    public:

    WheelOdomNode();

    private:

    ros::Time m_CurrentTime;
    ros::Time m_LastTime;

    ros::Publisher m_OdomPublisher;

    ros::Subscriber m_WheelEncoderSubscriber;

    nav_msgs::Odometry m_OldOdom;
    nav_msgs::Odometry m_NewOdom;

    const double m_WheelRadius = 0.1;
    const double m_WheelBase = 0.4;

    double m_WheelDistanceLeft;
    double m_WheelDistanceRight;
    double m_LastRightEncoder;
    double m_LastLeftEncoder;

    bool m_InitialEncoderReceived = false;
    double m_OldX;
    double m_OldY;
    double m_OldTheta;

    // Member Functions

    void update_odom();

    void calculateMovement(const geometry_msgs::Point::ConstPtr& encoders_rl);

    double normalize_angle(double angle);

};

#endif