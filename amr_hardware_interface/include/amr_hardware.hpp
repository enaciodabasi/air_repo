/**
 * @file amr_hardware.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef AMR_HARDWARE_HPP
#define AMR_HARDWARE_HPP

#include <ros/ros.h>
#include <ros/time.h>
#include <std_srvs/SetBool.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <controller_manager/controller_manager.h>

#include <iostream>
#include <vector>
#include <unordered_map>
#include <boost/shared_ptr.hpp>

#include "slave.hpp"
#include "master.hpp"
#include "domain.hpp"
#include "dc_sync.h"

namespace amr
{
    namespace hardware
    {
        class HardwareInterface : public hardware_interface::RobotHW
        {
            HardwareInterface(ros::NodeHandle& nh);
            ~HardwareInterface();

            void update(const ros::TimerEvent& timer_event);
            void read();
            void write(ros::Duration& elapsed_time);

            private:

            ethercat_interface::master::Master* m_Master;
            ethercat_interface::domain::Domain* m_Domain;

            std::vector<std::string> m_JointNames;

            std::size_t m_NumJoints; 

            std::vector<double> m_JointPositions;
            std::vector<double> m_JointVelocities;
            std::vector<double> m_JointEfforts;

            std::vector<double> m_VelocityCommands;

            ros::NodeHandle m_NodeHandle;

            ros::ServiceServer m_DriveStatusServer;
            bool m_DriverStatus = true;

            ros::Timer m_Loop;
            double m_LoopFrequency = 50;
            
            boost::shared_ptr<controller_manager::ControllerManager> m_ControllerManager;

            hardware_interface::JointStateInterface m_JointStateInterface;

            hardware_interface::VelocityJointInterface m_VelJointInterface;

            std::unordered_map<std::string, std::string> m_SymbolNameMap;

            void loadParams();

            bool callback_drive_status_change(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
        };
    }
}

#endif // AMR_HARDWARE_HPP