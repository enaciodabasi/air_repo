#ifndef COMMS_HANDLER_HPP
#define COMMS_HANDLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class CommsHandler
{
    public:

    CommsHandler(ros::NodeHandle& nh);

    void setVels(double linear_x, double angular_z);

    private:

    ros::NodeHandle m_NodeHandle;

    ros::Publisher m_TwistPublisher;

    void createAndPublishTwist(double linear_x, double angular_z);


};

#endif