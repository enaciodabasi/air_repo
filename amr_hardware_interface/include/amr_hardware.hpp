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
#include "utilities.hpp"

#include <thread>
#include <pthread.h>

#include "../include/amr_hwi_utility.hpp"


extern "C"
{
    #include "dc_sync.h"
}

using namespace ethercat_interface::utilities;


class ScheduledThread : public std::thread
{
    public:

    ScheduledThread() {}

    static void setScheduling(std::thread& thread_to_set, int policy, int priority)
    {
        sched_param schParam;
        schParam.sched_priority = priority;

        if(pthread_setschedparam(thread_to_set.native_handle(), policy, &schParam))
        {
            std::cerr << "Can't change std::thread scheduling." << std::strerror(errno) << std::endl;
        }

    }
    
};


namespace amr
{
    namespace hardware
    {
        class HardwareInterface : public hardware_interface::RobotHW
        {
            public:

            HardwareInterface(ros::NodeHandle& nh);
            ~HardwareInterface();

            std::shared_ptr<controller_manager::ControllerManager> cm;

            //void update(const ros::TimerEvent& timer_event);
            void update();
            void read();
            void write();

            ethercat_interface::master::Master* m_Master;
            ethercat_interface::domain::Domain* m_Domain;
            std::shared_ptr<ethercat_interface::logger::Logger> m_Logger;
            ros::Timer m_Loop;
            double m_LoopFrequency = 50;
            timespec m_CycleTime;
            long PERIOD_NS = 0;
            timespec m_WakeupTime;
            timespec m_Time;

            int m_ClockToUse = CLOCK_MONOTONIC;

            int ref_clock_counter = 0;

            debug::TimeMeasureInfo_s m_Measurer;
            
            public:

            std::vector<std::string> m_JointNames;

            std::size_t m_NumJoints; 

            std::vector<double> m_JointPositions;
            std::vector<double> m_JointVelocities;
            std::vector<double> m_JointEfforts;
            
            double m_WheelRadius = 0.1; // [m]

            int32_t m_TargetVelLeft = 0;
            int32_t m_TargetVelRight = 0;

            int32_t m_LeftWheelPos = 0;
            int32_t m_RightWheelPos = 0;
            
            public:
            std::vector<double> m_VelocityCommands;
            private:
            ros::NodeHandle m_NodeHandle;

            ros::ServiceServer m_DriveStatusServer;
            bool m_DriverStatus = true;
            //double m_WheelRadius = 0.1;
            //boost::shared_ptr<controller_manager::ControllerManager> m_ControllerManager;
            
            hardware_interface::JointStateInterface m_JointStateInterface;

            hardware_interface::VelocityJointInterface m_VelJointInterface;

            std::unordered_map<std::string, std::string> m_SymbolNameMap;
            public:
            utils::VelocityHelper m_DriverInfo;

            utils::PositionHelper m_PositionHelper; 

            void loadParams();

            //bool callback_drive_status_change(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
        };
    }
}

#endif // AMR_HARDWARE_HPP