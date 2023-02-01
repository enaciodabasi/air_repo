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
#include <controller_manager/controller_manager.h>

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

            typedef std::pair<std::vector<std::string>, std::vector<std::string>> NameSymbolPair; 

            public:

            AirHardwareInterface(ros::NodeHandle& nh);

            ~AirHardwareInterface();

            void update(const ros::TimerEvent& te);

            void write(const ros::Duration& elapsed_time);

            void read();

            private:

            // ----- Private member variables -----

            ros::NodeHandle m_NodeHandle;

            ros::Timer m_NonRealtimeLoop;

            io::ads_interface* m_AdsInterface;

            RightArm* m_RightArm;

            LeftArm* m_LeftArm;

            int m_TotalNumberOfJoints;

            hardware_interface::JointStateInterface m_JointStateInterface;

            hardware_interface::PosVelJointInterface m_PosVelJointInterface;

            double m_LoopFrequency;

            boost::shared_ptr<controller_manager::ControllerManager> m_ControllerManager;

            // ----- Private member functions -----

            RobotParams loadRobotParams();

            io::AdsInfo loadAdsParams();

            NameSymbolPair loadSymbols(const std::string& arm_name);

        };
    }
}


#endif // AIR_HARDWARE_INTERFACE_HPP