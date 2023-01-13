#ifndef AIR_ARM_HARDWARE_INTERFACE_HPP
#define AIR_ARM_HARDWARE_INTERFACE_HPP

#include "air_arm_hardware.hpp"
#include "air_right_arm_interface.hpp"

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

namespace air
{
    class RightArmHardwareInterface : public RightArmHardware
    {
        public:

        RightArmHardwareInterface(ros::NodeHandle& nh);
        ~RightArmHardwareInterface();

        void init();
        void update(const ros::TimerEvent& te);
        void read();
        void write(const ros::Duration& elapsed_time);

        protected:

        RightArm m_RightArm;

        ros::NodeHandle m_NodeHandle;

        ros::Timer m_NonRealtimeLoop;

        ros::Duration m_ControlPeriod;
        ros::Duration m_ElapsedTime;

        hardware_interface::PosVelAccJointInterface m_PositionJointInterface;

        double m_LoopFrequency; // in Hz
        
        boost::shared_ptr<controller_manager::ControllerManager> m_ControllerManager;

        double m_pError;
        double m_vError;
        double m_eError;

        std::string m_LogInfo;

    } ;
}

#endif
