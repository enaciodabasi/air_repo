/**
 * @file amr_hardware.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/amr_hardware.hpp"

namespace amr
{
    namespace hardware
    {
        HardwareInterface::HardwareInterface(ros::NodeHandle& nh)
            : m_NodeHandle{nh}
        {
            this->loadParams();

            m_DriveStatusServer = m_NodeHandle.advertiseService(
                "change_drive_status",
                &HardwareInterface::callback_drive_status_change,
                this
            );

            m_NumJoints = m_JointNames.size();

            m_JointPositions.resize(m_NumJoints);
            m_JointVelocities.resize(m_NumJoints);
            m_JointEfforts.resize(m_NumJoints);

            m_VelocityCommands.resize(m_NumJoints);

            for(std::size_t i = 0; i < m_NumJoints; i++)
            {
                hardware_interface::JointStateHandle jointStateHandle(
                    m_JointNames.at(i),
                    &m_JointPositions.at(i),
                    &m_JointVelocities.at(i),
                    &m_JointEfforts.at(i)
                );

                m_JointStateInterface.registerHandle(jointStateHandle);

                hardware_interface::JointHandle jointHandle(
                    jointStateHandle,
                    &m_VelocityCommands.at(i)
                );
                m_VelJointInterface.registerHandle(jointHandle);
            }

            this->registerInterface(&m_JointStateInterface);
            this->registerInterface(&m_VelJointInterface);

            m_ControllerManager.reset(new controller_manager::ControllerManager(this, m_NodeHandle));

            ros::Duration updateFrequency = ros::Duration(1.0 / m_LoopFrequency);
            m_Loop = m_NodeHandle.createTimer(
                updateFrequency,
                &HardwareInterface::update,
                this
            );

        }

        HardwareInterface::~HardwareInterface()
        {

        }

        void HardwareInterface::update(const ros::TimerEvent& timer_event)
        {
            this->read();

            ros::Duration elapsedTime = ros::Duration(timer_event.current_real - timer_event.last_real);

            m_ControllerManager->update(timer_event.current_real, elapsedTime);

            this->write(elapsedTime);
        }

        void HardwareInterface::write(ros::Duration& elapsed_time)
        {
            ros::Duration elapsedTime = elapsed_time;

            
        }

        void HardwareInterface::read()
        {
    
            
            
            double leftWheelVel = 0.0;
            double leftWheelPos = 0.0;
            double rightWheelVel = 0.0;
            double rightWheelPos = 0.0;
            
            m_JointPositions[0] = leftWheelPos;
            
            m_JointVelocities[0] = leftWheelVel;
            
            m_JointEfforts[0] = 0.0;
            
            m_JointPositions[1] = rightWheelPos;
            
            m_JointVelocities[1] = rightWheelVel;
            
            m_JointEfforts[1] = 0.0;
            
            

        }

        bool HardwareInterface::callback_drive_status_change(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
        {

            return true;
        }

        void HardwareInterface::loadParams()
        {
            if(m_NodeHandle.hasParam("/amr/hardware_interface/joints"))
            {
                m_NodeHandle.getParam("/amr/hardware_interface/joints", m_JointNames);
            }
            else
            {
                ROS_ERROR("No joint names find in the parameter server. Shutting down the hardware interface.");
                if(ros::ok())
                    ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/amr/harware_interface/loop_hz"))
            {
                m_NodeHandle.getParam("/amr/hardware_interface/loop_hz", m_LoopFrequency);
            }
            else
            {
                ROS_INFO("Loop freuency is not specified in the parameter server. Defaulting back to 50 Hz");
            }
                  
        }    
        
    
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "amr_hardware_interface_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}