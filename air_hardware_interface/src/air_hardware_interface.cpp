/**
 * @file air_hardware_interface.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "../include/air_hardware_interface.hpp"

namespace air
{
    namespace hwi
    {
        AirHardwareInterface::AirHardwareInterface(ros::NodeHandle& nh)
        {
            RobotParams robotParams = loadRobotParams();
            io::AdsInfo adsParams = loadAdsParams();

            m_AdsInterface = new io::ads_interface(
                adsParams.remote_ipv4,
                adsParams.remote_netId,
                adsParams.local_address,
                adsParams.port_num
            );

            m_RightArm = new RightArm(robotParams.jointNamesRightArm);
            auto p = loadSymbols("right_arm");
            m_RightArm->init_ads_symbol_map(p.first, p.second);
            m_LeftArm = new LeftArm(robotParams.jointNamesLeftArm);
            p = loadSymbols("left_arm");
            m_LeftArm->init_ads_symbol_map(p.first, p.second);

            m_TotalNumberOfJoints = (int)m_RightArm->m_NumOfJoints + (int)m_LeftArm->m_NumOfJoints;

            for(int i = 0; i < m_RightArm->m_NumOfJoints; i++)
            {
                hardware_interface::JointStateHandle jointStateHandle(
                    m_RightArm->m_JointNames[i],
                    &m_RightArm->m_JointPositions[i],
                    &m_RightArm->m_JointVelocities[i],
                    &m_RightArm->m_JointEfforts[i]
                );

                m_JointStateInterface.registerHandle(jointStateHandle);

                hardware_interface::PosVelJointHandle posVelJointHandle(
                    jointStateHandle,
                    &m_RightArm->m_PosCmds[i],
                    &m_RightArm->m_VelCmds[i]
                );

                m_PosVelJointInterface.registerHandle(posVelJointHandle);
            }

            int j = 0;
            while(j < m_LeftArm->m_NumOfJoints)
            {
                hardware_interface::JointStateHandle jointStateHandle(
                    m_LeftArm->m_JointNames[j],
                    &m_LeftArm->m_JointPositions[j],
                    &m_LeftArm->m_JointVelocities[j],
                    &m_LeftArm->m_JointEfforts[j]
                );

                m_JointStateInterface.registerHandle(jointStateHandle);

                hardware_interface::PosVelJointHandle posVelJointHandle(
                    jointStateHandle,
                    &m_LeftArm->m_PosCmds[j],
                    &m_LeftArm->m_VelCmds[j]
                );

                m_PosVelJointInterface.registerHandle(posVelJointHandle);

                j += 1;
            }

            this->registerInterface(&m_JointStateInterface);
            this->registerInterface(&m_PosVelJointInterface);

            m_ControllerManager.reset(new controller_manager::ControllerManager(this, m_NodeHandle));

            m_NodeHandle.param("air_arm/hardware_interface/loop_hz", m_LoopFrequency, 50.0);

            ros::Duration updatePeriod = ros::Duration(1.0 / m_LoopFrequency);

            m_NonRealtimeLoop = m_NodeHandle.createTimer(
                updatePeriod,
                &AirHardwareInterface::update,
                this
            );

        }

        AirHardwareInterface::~AirHardwareInterface()
        {
            
        }

        void AirHardwareInterface::update(const ros::TimerEvent& te)
        {
            ros::Duration elapsedTime = ros::Duration(te.current_real - te.last_real);

            this->read();

            m_ControllerManager->update(te.current_real, elapsedTime);

            this->write(elapsedTime);


        }

        void AirHardwareInterface::write(const ros::Duration& elapsed_time)
        {
            m_RightArm->write(
                m_RightArm->m_PosCmds,
                m_RightArm->m_VelCmds
            );

            // Write to left arm:

        }

        void AirHardwareInterface::read()
        {
            auto actJointStates_R = m_RightArm->read();
            auto actJointStates_L = m_LeftArm->read();

            for(std::size_t i = 0; i < actJointStates_R.position.size(); i++)
            {
                m_RightArm->m_JointPositions.at(i) = actJointStates_R.position.at(i);
                m_RightArm->m_JointVelocities.at(i) = actJointStates_R.velocity.at(i);
            }

            for(std::size_t i = 0; i < actJointStates_L.position.size(); i++)
            {
                m_LeftArm->m_JointPositions.at(i) = actJointStates_L.position.at(i);
                m_LeftArm->m_JointVelocities.at(i) = actJointStates_L.velocity.at(i);
            }
        }


        RobotParams AirHardwareInterface::loadRobotParams()
        {   
            RobotParams robotParams;

            if(m_NodeHandle.hasParam("/air_arm/joints/right_arm"))
            {
                m_NodeHandle.getParam("/air_arm/joints/right_arm", robotParams.jointNamesRightArm);
            }
            else  
            {
                ROS_ERROR("No right arm joint names find in the parameter server. Shutting down the hardware interface.");
                if(ros::ok())
                    ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/air_arm/joints/left_arm"))
            {
                m_NodeHandle.getParam("/air_arm/joints/left_arm", robotParams.jointNamesLeftArm);
            }
            else
            {
                ROS_ERROR("No left arm joint names find in the parameter server. Shutting down the hardware interface.");
                if(ros::ok())
                    ros::shutdown();
            }

            return robotParams;
        }

        io::AdsInfo AirHardwareInterface::loadAdsParams()
        {
            io::AdsInfo adsParams;

            if(m_NodeHandle.hasParam("/air_arm/ads_config/remote_ipv4"))
            {
                m_NodeHandle.getParam("/air_arm/ads_config/remote_ipv4", adsParams.remote_ipv4);
            }
            else
            {
                ROS_ERROR("Can't find the Remote IpV4 Addres in the parameter server.");
                ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/air_arm/ads_config/remote_net_id"))
            {
                m_NodeHandle.getParam("/air_arm/ads_config/remote_net_id", adsParams.remote_netId);
            }
            else
            {
                ROS_ERROR("Can't find the Remote NetID in the parameter server.");
                ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/air_arm/ads_config/local_adress"))
            {   
                std::vector<int> tempLocalAddr;
                m_NodeHandle.getParam("/air_arm/ads_config/local_adress", tempLocalAddr);

                std::array<uint8_t, 6> arr = [tempLocalAddr](){
                    std::array<uint8_t, 6> temp;
                    for(std::size_t i = 0; i < tempLocalAddr.size(); i++)
                    {
                        temp[i] = (uint8_t)tempLocalAddr[i];
                    }   
                    return temp;
                }();

                adsParams.local_address = arr;
            }
            else
            {
                ROS_ERROR("Can't find the Local Addres in the parameter server.");
                ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/air_arm/ads_config/port_num"))
            {
                int tempPort;
                m_NodeHandle.getParam("/air_arm/ads_config/port_num", tempPort);
                adsParams.port_num = (uint16_t)tempPort;
            }
            else
            {
                ROS_INFO("Could not find port number in the parameter server. Defaulting back to 851.");
            }

            return adsParams;
        }

        AirHardwareInterface::NameSymbolPair AirHardwareInterface::loadSymbols(const std::string& arm_name)
        {
            const std::string param_path = "/air_arm/ads_config/symbols/" + arm_name;

            std::vector<std::string> names;
            std::vector<std::string> symbols;

            // 
            if(m_NodeHandle.hasParam(param_path + "/goal_pos"))
            {   
                std::string tempStr;
                m_NodeHandle.getParam(param_path + "/goal_pos", tempStr);
                names.push_back("goal_pos");
                symbols.push_back(tempStr);
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the pos goal. Shutting down...");
                    ros::shutdown();
            }  

            if(m_NodeHandle.hasParam(param_path + "/goal_vel"))
            {   
                std::string tempStr;
                m_NodeHandle.getParam(param_path + "/goal_vel", tempStr);
                names.push_back("goal_vel");
                symbols.push_back(tempStr);
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the velocity goal. Shutting down...");
                    ros::shutdown();
            } 

            if(m_NodeHandle.hasParam(param_path + "/act_pos"))
            {   
                std::string tempStr;
                m_NodeHandle.getParam(param_path + "/act_pos", tempStr);
                names.push_back("act_pos");
                symbols.push_back(tempStr);
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the actual pos. Shutting down...");
                    ros::shutdown();
            } 

            if(m_NodeHandle.hasParam(param_path + "/act_vel"))
            {   
                std::string tempStr;
                m_NodeHandle.getParam(param_path + "/act_vel", tempStr);
                names.push_back("act_vel");
                symbols.push_back(tempStr);
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the actual velocity. Shutting down...");
                    ros::shutdown();
            } 


            if(m_NodeHandle.hasParam(param_path + "/act_time"))
            {   
                std::string tempStr;
                m_NodeHandle.getParam(param_path + "/act_time", tempStr);
                names.push_back("act_time");
                symbols.push_back(tempStr);
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the actual timestamp. Shutting down...");
                    ros::shutdown();
            } 

            if(m_NodeHandle.hasParam(param_path + "/last_time"))
            {   
                std::string tempStr;
                m_NodeHandle.getParam(param_path + "/last_time", tempStr);
                names.push_back("last_time");
                symbols.push_back(tempStr);
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the last timestamp. Shutting down...");
                    ros::shutdown();
            } 


            if(m_NodeHandle.hasParam(param_path + "/halt"))
            {   
                std::string tempStr;
                m_NodeHandle.getParam(param_path + "/halt", tempStr);
                names.push_back("halt");
                symbols.push_back(tempStr);
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the HALT STATE. Shutting down...");
                    ros::shutdown();
            } 

            if(m_NodeHandle.hasParam(param_path + "/timeout"))
            {   
                std::string tempStr;
                m_NodeHandle.getParam(param_path + "/timeout", tempStr);
                names.push_back("timeout");
                symbols.push_back(tempStr);
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the TIMEOUT STATE. Shutting down...");
                    ros::shutdown();
            } 

            return NameSymbolPair(names, symbols);
            
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "air_hardware_interface_node");
    
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    air::hwi::AirHardwareInterface airHwi(nh);

    ros::waitForShutdown();

    return 0;
}