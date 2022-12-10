#ifndef AIR_ARM_HARDWARE_HPP
#define AIR_ARM_HARDWARE_HPP

#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <sensor_msgs/JointState.h>

#include <pluginlib/class_list_macros.hpp>

#include <boost/scoped_ptr.hpp>

#include "AdsLib.h"
#include "AdsVariable.h"


namespace air
{
    class RightArmHardware : public hardware_interface::RobotHW
    {
        protected:

        ros::NodeHandle m_NodeHandle;

        hardware_interface::JointStateInterface m_JointStateInterface;
        hardware_interface::PosVelAccJointInterface m_PosVelAccJointInterface;

        int m_NumJoints;
        std::vector<std::string> m_JointNames;

        std::vector<double> m_JointPositions;
        std::vector<double> m_JointVelocities;
        std::vector<double> m_JointEfforts;

        std::vector<double> m_PositionCommand;
        std::vector<double> m_VelocityCommand;
        std::vector<double> m_AccelCommand;

    };
}

#endif