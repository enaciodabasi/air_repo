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
#include "../include/offset_el7221_9014.hpp"

namespace amr
{
    namespace hardware
    {
        using namespace ethercat_interface;

        HardwareInterface::HardwareInterface(ros::NodeHandle& nh)
            : m_NodeHandle{nh}
        {
            this->loadParams();
            m_JointNames[0] = "lw_joint";
            m_JointNames[1] = "rw_joint";
            ROS_INFO("Loaded parameters.");
            PERIOD_NS = period_nanosec(500);
            ROS_INFO("PERIOS_NS");
            m_CycleTime = {0, PERIOD_NS};

            //const auto sysuser = getenv("USER");
            m_Logger = std::make_shared<logger::Logger>("/home/naci/catkin_ws/src/amr_hardware_interface/logs/", logger::FILE);
            ROS_INFO("Creating master");
            m_Master = new master::Master(0, m_Logger);
            ROS_INFO("Created master");
            m_Domain = new domain::Domain("amr_domain", m_Logger);
            ROS_INFO("Created domain");
            m_Master->registerDomain(m_Domain);
            ROS_INFO("Registered domain.");

            m_Domain->registerSlave(
                new slave::Slave(
                    "EK1100_0",
                    "/home/naci/catkin_ws/src/amr_hardware_interface/config/amr_config.yaml",
                    nullptr,
                    m_Logger,
                    false
                )
            );
            ROS_INFO("Created Slave EK1100");
            m_Domain->registerSlave(
                new slave::Slave(
                    "EL7221_9014_0",
                    "/home/naci/catkin_ws/src/amr_hardware_interface/config/amr_config.yaml",
                    new EL7221_9014_Offset(),
                    m_Logger,
                    true
                )
            );
            ROS_INFO("Created Slave EL7221_9014_0");

            m_Domain->registerSlave(
                new slave::Slave(
                    "EL7221_9014_1",
                    "/home/naci/catkin_ws/src/amr_hardware_interface/config/amr_config.yaml",
                    new EL7221_9014_Offset(),
                    m_Logger,
                    true
                )
            );
            ROS_INFO("Created Slave EL7221_9014_1");

            /* m_DriveStatusServer = m_NodeHandle.advertiseService(
                "change_drive_status",
                &HardwareInterface::callback_drive_status_change,
                this
            ); */

            m_Master->configureDomains();
            m_Master->setupDomains();

            if(!m_Master->activateMaster())
            {
                ROS_FATAL("Can't activate master.");
                ros::shutdown();
            }

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

            clock_gettime(m_ClockToUse, &m_WakeupTime);

            ros::Duration updateFrequency = ros::Duration(0.002);
            m_Loop = m_NodeHandle.createTimer(
                updateFrequency,
                &HardwareInterface::update,
                this
            );

        }

        HardwareInterface::~HardwareInterface()
        {
            delete m_Domain;
            delete m_Master;
        }

        void HardwareInterface::update(const ros::TimerEvent& timer_event)
        {
            m_WakeupTime = addTimespec(m_WakeupTime, m_CycleTime);
            sleep_task(m_ClockToUse, TIMER_ABSTIME, &m_WakeupTime, NULL);
            m_Master->setMasterTime(timespecToNanoSec(m_WakeupTime));
            m_Master->receive("amr_domain");

            m_Master->updateMasterState();
            m_Master->updateDomainStates();
            m_Master->updateSlaveStates();
            
            bool slavesEnabled = m_Master->enableSlaves();

            m_Master->write<int8_t>(
                "amr_domain",
                "EL7221_9014_0",
                "op_mode",
                0x09
            );

            m_Master->write<int8_t>(
                "amr_domain",
                "EL7221_9014_1",
                "op_mode",
                0x09
            );

            if(slavesEnabled)
            {   
                ROS_INFO("Slaves enabled");
                this->read();

                ros::Duration elapsedTime = ros::Duration(timer_event.current_real - timer_event.last_real);

                m_ControllerManager->update(timer_event.current_real, elapsedTime);

                this->write(elapsedTime);

                
            }
            m_Master->syncMasterClock(timespecToNanoSec(m_Time));
            m_Master->send("amr_domain");
        }

        void HardwareInterface::write(ros::Duration& elapsed_time)
        {
            ros::Duration elapsedTime = elapsed_time;

            int32_t targetVelLeft = utils::linearVelToDriverCmd(
                m_VelocityCommands[0],
                m_DriverInfo
            );

            int32_t targetVelRight = utils::linearVelToDriverCmd(
                m_VelocityCommands[1],
                m_DriverInfo
            );

            m_Master->write<int32_t>(
                "amr_domain",
                "EL7221_9014_0",
                "target_velocity",
                targetVelLeft
            );
            // sag
            m_Master->write<int32_t>(
                "amr_domain",
                "EL7221_9014_1",
                "target_velocity",
                targetVelRight
            );
            
        }

        void HardwareInterface::read()
        {
            
            double leftWheelVel = (double)(m_Master->read<int32_t>("amr_domain", "EL7221_9014_0", "current_velocity"));
            double leftWheelPos = 0.0;
            double rightWheelVel = (double)(m_Master->read<int32_t>("amr_domain", "EL7221_9014_1", "current_velocity"));
            double rightWheelPos = 0.0;

            m_JointPositions[0] = leftWheelPos;
            
            m_JointVelocities[0] = utils::linearVelToDriverCmd(leftWheelVel, m_DriverInfo);
            
            m_JointEfforts[0] = 0.0;
            
            m_JointPositions[1] = rightWheelPos;
            
            m_JointVelocities[1] = utils::linearVelToDriverCmd(rightWheelVel, m_DriverInfo);
            
            m_JointEfforts[1] = 0.0;
            
            

        }

        /* bool HardwareInterface::callback_drive_status_change(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
        {

            return true;
        } */

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

            if(m_NodeHandle.hasParam("/amr/driver_info"))
            {   
                m_NodeHandle.getParam("/amr/driver_info/velocity_enc_resolution", m_DriverInfo.velocityEncoderResolution);
                m_NodeHandle.getParam("/amr/driver_info/wheel_side_gear", m_DriverInfo.wheelSideGear);
                m_NodeHandle.getParam("/amr/driver_info/motor_side_gear", m_DriverInfo.motorSideGear);
                m_NodeHandle.getParam("/amr/driver_info/motor_gear_heat", m_DriverInfo.motorGearHeat);
                m_NodeHandle.getParam("/amr/driver_info/wheel_diameter", m_DriverInfo.wheelDiameter);
                m_NodeHandle.getParam("/amr/driver_info/motor_max_rpm", m_DriverInfo.motorMaxRPM);
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
    amr::hardware::HardwareInterface hw(nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}