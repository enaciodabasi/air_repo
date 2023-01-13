#ifndef WHEEL_ODOM_SERVER_HPP
#define WHEEL_ODOM_SERVER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <math.h>
#include <iostream>

#include "amr_custom_interfaces/OdomSrv.h"

namespace amr
{

    using amr_custom_interfaces::OdomSrv;

    namespace localization
    {

        class OdomServer
        {
            public:

            OdomServer(ros::NodeHandle& nh);

            private:

            ros::NodeHandle m_NodeHandle;

            ros::Time m_CurrentTime;
            
            ros::Time m_LastTime;

            ros::ServiceServer m_OdomServer;

            nav_msgs::Odometry m_OldOdom;

            nav_msgs::Odometry m_NewOdom;

            double m_WheelRadius;

            double m_WheelBase;

            double m_WheelDistanceLeft;

            double m_WheelDistanceRight;

            double m_PrevRightEncoder;

            double m_PrevLeftEncoder;

            bool m_InitialEncoderReceived = false;

            double m_OldX;

            double m_OldY;

            double m_OldTheta;

            double normalize_angle(double angle);

            void update_odom();

            bool callback_odom_server(OdomSrv::Request &request, OdomSrv::Response& response);

            void loadParams();


        };
    }
}


#endif