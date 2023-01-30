#include "../include/amr_localization/wheel_odom_server.hpp"

namespace amr
{   
    using amr_custom_interfaces::OdomSrv;

    namespace localization
    {
        OdomServer::OdomServer(ros::NodeHandle& nh)
            : m_NodeHandle(nh)
        {

            // Load parameters from parameter server.

            loadParams();

            // Initialize Service Server for odometry.
            m_OdomServer = m_NodeHandle.advertiseService(
                "odom_server",
                &OdomServer::callback_odom_server,
                this
            );


        }
        bool OdomServer::callback_odom_server(OdomSrv::Request& request, OdomSrv::Response& response)
        {
            double rightEncoder = request.encoders.x;
            double leftEncoder = request.encoders.y;

            if(m_InitialEncoderReceived)
            {
                m_CurrentTime = ros::Time::now();

                m_WheelDistanceRight = (rightEncoder - m_PrevRightEncoder) / 1000.0;
                m_WheelDistanceLeft = (leftEncoder - m_PrevLeftEncoder) / 1000.0;
                this->update_odom();

                response.odom = m_NewOdom;
            }
            else
            {
                m_InitialEncoderReceived = true;
                m_CurrentTime = ros::Time::now();
                m_LastTime = m_CurrentTime;
            }

            m_PrevLeftEncoder = leftEncoder;
            m_PrevRightEncoder = rightEncoder;

            return true;
        
        }

        void OdomServer::update_odom()
        {
            
            double cycleDistance = (m_WheelDistanceRight + m_WheelDistanceLeft) / 2.0;
            double cycleAngle = std::asin((m_WheelDistanceRight - m_WheelDistanceLeft) / m_WheelBase);
            double averageAngle = (cycleAngle / 2.0) + m_OldTheta;

            double averageAngle_Normalized = normalize_angle(averageAngle);

            m_NewOdom.pose.pose.position.x = m_OldX + std::cos(averageAngle_Normalized) * cycleDistance;
            m_NewOdom.pose.pose.position.y = m_OldY + std::sin(averageAngle_Normalized) * cycleDistance;
            m_NewOdom.pose.pose.position.z = 0;

            m_OldTheta += cycleAngle;
            m_OldTheta = normalize_angle(m_OldTheta);

            tf2::Quaternion quat;
            quat.setRPY(0, 0, m_OldTheta);
            m_NewOdom.pose.pose.orientation.x = quat.x();
            m_NewOdom.pose.pose.orientation.y = quat.y();
            m_NewOdom.pose.pose.orientation.z = quat.z();
            m_NewOdom.pose.pose.orientation.w = quat.w();

            m_LastTime = ros::Time::now();

            double dt = (m_CurrentTime - m_LastTime).toSec();

            m_NewOdom.header.stamp = m_LastTime;
            m_NewOdom.twist.twist.linear.x = cycleDistance/dt;
            m_NewOdom.twist.twist.linear.y = 0;
            m_NewOdom.twist.twist.linear.z = 0;
            m_NewOdom.twist.twist.angular.x = 0;
            m_NewOdom.twist.twist.angular.y = 0;
            m_NewOdom.twist.twist.angular.z = cycleAngle/dt;

            m_NewOdom.header.frame_id = "odom";

            m_OldX = m_NewOdom.pose.pose.position.x;
            m_OldY = m_NewOdom.pose.pose.position.y;
            m_OldOdom = m_NewOdom;

        }   

        double OdomServer::normalize_angle(double angle)
        {
            if(angle > M_PI)
            {
                return (angle -= 2*M_PI);
            }
            else if(angle < -M_PI)
            {
                return (angle += 2*M_PI);
            }

            return angle;
        }

        void OdomServer::loadParams()
        {
            if(m_NodeHandle.hasParam("/amr/wheel/radius"))
            {
                m_NodeHandle.getParam("/amr/wheel/radius", m_WheelRadius);
            }
            else
            {
                ROS_INFO("Couldn't find wheel radius in the parameter server. Defaulting back to 0.1");
                m_WheelRadius = 0.2;
            }

            if(m_NodeHandle.hasParam("/amr/wheel/base"))
            {
                m_NodeHandle.getParam("/amr/wheel/base", m_WheelBase);
            }
            else
            {
                ROS_INFO("Couldn't find wheel base in the parameter server, defaulting back to 0.4");
                m_WheelBase = 0.4;
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_service_node");
    ros::NodeHandle nh;
    amr::localization::OdomServer odomServerNode(nh);

    ros::spin();

    return 0;
}