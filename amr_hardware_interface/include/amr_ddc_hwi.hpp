#ifndef AMR_DDC_HWI_HPP
#define AMR_DDC_HWI_HPP

#include "io_ads.hpp"
#include "amr_hwi_utility.hpp"

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

namespace amr
{
    namespace controlled_hwi
    {
        
        class HardwareInterface: public hardware_interface::RobotHW
        {
            public:

            HardwareInterface(ros::NodeHandle& nh, bool use_ticks = true);
            ~HardwareInterface();

            void update(const ros::TimerEvent& timer_event);
            void read();
            void write(ros::Duration& elapsed_time);

            private:

            bool m_UseTicks;

            std::vector<std::string> m_JointNames;

            std::size_t m_NumJoints;

            utils::WheelState* m_LeftWheelState;

            utils::WheelState* m_RightWheelState;  

            int m_EncoderResolution;          

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

            std::unique_ptr<io::ads_interface> m_AdsInterface;

            io::AdsInfo m_AdsInfo;

            std::unordered_map<std::string, std::string> m_SymbolNameMap;

            void loadParams();

            bool callback_drive_status_change(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

        };
    }
}

#endif
