#ifndef COMMS_HANDLER_HPP
#define COMMS_HANDLER_HPP

#include <ros/ros.h>

class CommsHandler
{
    public:

    CommsHandler(ros::NodeHandle& nh);

    private:

    ros::NodeHandle m_NodeHandle;


};

#endif