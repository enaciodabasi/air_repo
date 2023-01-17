#include "../include/comms_handler.hpp"

CommsHandler::CommsHandler(ros::NodeHandle& nh)
    : m_NodeHandle(nh)
{
    m_TwistPublisher = nh.advertise<geometry_msgs::Twist>(
        "cmd_vel",
        10
    );

    m_DriveStatusClient = m_NodeHandle.serviceClient<std_srvs::SetBool>("change_drive_status");
}

void CommsHandler::setVels(double linear_x, double angular_z)
{
    createAndPublishTwist(linear_x, angular_z);
}

void CommsHandler::createAndPublishTwist(double linear_x, double angular_z)
{
    geometry_msgs::Twist velCmd;
    velCmd.linear.x = linear_x;
    velCmd.linear.y = 0.0;
    velCmd.linear.z = 0.0;

    velCmd.angular.x = 0.0;
    velCmd.angular.y = 0.0;
    velCmd.angular.z = angular_z;

    ROS_INFO("x: %f | y: %f", linear_x, angular_z);

    m_TwistPublisher.publish(velCmd);

}

bool CommsHandler::changeDriveStatus(bool new_drive_status)
{
    std_srvs::SetBool setBoolSrv;
    setBoolSrv.request.data = new_drive_status;
    
    if(m_DriveStatusClient.call(setBoolSrv))
    {   
        return true;
    }

    return false;
}