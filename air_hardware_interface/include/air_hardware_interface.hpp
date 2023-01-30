/**
 * @file air_hardware_interface.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef AIR_HARDWARE_INTERFACE_HPP
#define AIR_HARDWARE_INTERFACE_HPP

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/posvel_command_interface.h>

#include "air_right_arm.hpp"
#include "air_left_arm.hpp"

namespace air
{
    namespace hwi
    {

        struct RobotParams
        {
            std::vector<std::string> jointNamesRightArm;
            std::vector<std::string> jointNamesLeftArm;

        };

        class AirHardwareInterface : public hardware_interface::RobotHW
        {
            public:

            AirHardwareInterface(ros::NodeHandle& nh);

            ~AirHardwareInterface();

            private:

            ros::NodeHandle m_NodeHandle;

            RightArm* m_RightArm;

            LeftArm* m_LeftArm;

            int m_TotalNumberOfJoints;

            hardware_interface::JointStateInterface m_JointStateInterface;

            hardware_interface::PosVelJointInterface m_PosVelJointInterface;

            RobotParams loadRobotParams();

            io::AdsInfo loadAdsParams();

        };
    }
}


#endif // AIR_HARDWARE_INTERFACE_HPP