#ifndef COMMS_HANDLER_HPP
#define COMMS_HANDLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>

class CommsHandler
{
    public:

    CommsHandler(ros::NodeHandle& nh);

    void setVels(double linear_x, double angular_z);

    bool changeDriveStatus(bool new_drive_status);

    private:

    ros::NodeHandle m_NodeHandle;

    ros::Publisher m_TwistPublisher;

    ros::ServiceClient m_DriveStatusClient;

    void createAndPublishTwist(double linear_x, double angular_z);

    bool m_CurrentState = true;

};

#endif