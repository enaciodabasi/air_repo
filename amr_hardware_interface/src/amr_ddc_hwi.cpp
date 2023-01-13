#include "../include/amr_ddc_hwi.hpp"

namespace amr
{
    namespace controlled_hwi
    {
        HardwareInterface::HardwareInterface(ros::NodeHandle& nh)
            : m_NodeHandle{nh}
        {
            this->loadParams();

            m_LeftWheelState = new utils::WheelState(
                m_JointNames.at(0),
                m_EncoderResolution
            );

            m_RightWheelState = new utils::WheelState(
                m_JointNames.at(1),
                m_EncoderResolution
            );

            m_AdsInterface = std::make_unique<io::ads_interface>(
                m_AdsInfo.remote_ipv4,
                m_AdsInfo.remote_netId,
                m_AdsInfo.local_address,
                m_AdsInfo.port_num
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
            if(m_LeftWheelState)
            {
                delete m_LeftWheelState;
            }

            if(m_RightWheelState)
            {
                delete m_LeftWheelState;
            }
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

            try
            {
                std::weak_ptr<AdsDevice> routeWeakPtr =  m_AdsInterface->getRoute();
                std::shared_ptr<AdsDevice> routeSharedPtr = routeWeakPtr.lock();

                AdsVariable<double> leftMotorAds{*routeSharedPtr, m_SymbolNameMap["left_motor"]};
                AdsVariable<double> rightMotorAds{*routeSharedPtr, m_SymbolNameMap["right_motor"]};

                leftMotorAds = m_VelocityCommands[0];
                rightMotorAds = m_VelocityCommands[1];
            }
            catch(AdsException& ex)
            {
                std::cout << ex.what();
            }
            catch(std::system_error& ex)
            {
                std::cout << ex.what();
            }
        }

        void HardwareInterface::read()
        {
        
            long encoderLeft = 0;
            long encoderRight = 0;

            // Get encoder ticks from Beckhoff via ADS.
            try
            {   
                std::weak_ptr<AdsDevice> routeWeakPtr =  m_AdsInterface->getRoute();
                std::shared_ptr<AdsDevice> routeSharedPtr = routeWeakPtr.lock();

                AdsVariable<long> leftEncoderAds{*routeSharedPtr, m_SymbolNameMap["left_encoder"]};
                AdsVariable<long> rightEncoderAds{*routeSharedPtr, m_SymbolNameMap["right_encoder"]};

                encoderLeft = leftEncoderAds;
                encoderRight = rightEncoderAds;

            }
            catch(AdsException& ex)
            {
                std::cout << ex.what();
            }
            catch(std::system_error& ex)
            {
                std::cout << ex.what();
            }

            ros::Time currentTime = ros::Time::now();
            utils::State* statePtr;
            *statePtr = m_LeftWheelState->getState(currentTime, encoderLeft);
            
            m_JointPositions[0] = statePtr->angularPos;
            m_JointVelocities[0] = statePtr->angularVel;
            m_JointEfforts[0] = 0.0;

            *statePtr = m_RightWheelState->getState(currentTime, encoderRight);

            m_JointPositions[1] = statePtr->angularPos;
            m_JointPositions[1] = statePtr->angularVel;
            m_JointEfforts[1] = 0.0;

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

            if(m_NodeHandle.hasParam("/amr/ads_config/remote_ipv4"))
            {
                m_NodeHandle.getParam("/amr/ads_config/remote_ipv4", m_AdsInfo.remote_ipv4);
            }
            else
            {
                ROS_ERROR("Can't find the Remote IpV4 Addres in the parameter server.");
                ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/amr/ads_config/remote_net_id"))
            {
                m_NodeHandle.getParam("/amr/ads_config/remote_net_id", m_AdsInfo.remote_netId);
            }
            else
            {
                ROS_ERROR("Can't find the Remote NetID in the parameter server.");
                ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/amr/ads_config/local_adress"))
            {   
                std::vector<int> tempLocalAddr;
                m_NodeHandle.getParam("/amr/ads_config/local_adress", tempLocalAddr);

                std::array<uint8_t, 6> arr = [tempLocalAddr](){
                    std::array<uint8_t, 6> temp;
                    for(std::size_t i = 0; i < tempLocalAddr.size(); i++)
                    {
                        temp[i] = (uint8_t)tempLocalAddr[i];
                    }   
                    return temp;
                }();

                m_AdsInfo.local_address = arr;
            }
            else
            {
                ROS_ERROR("Can't find the Local Addres in the parameter server.");
                ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/amr/ads_config/port_num"))
            {
                int tempPort;
                m_NodeHandle.getParam("/amr/ads_config/port_num", tempPort);
                m_AdsInfo.port_num = (uint16_t)tempPort;
            }
            else
            {
                ROS_INFO("Could not find port number in the parameter server. Defaulting back to 851.");
            }

            if(m_NodeHandle.hasParam("/amr/harware_interface/loop_hz"))
            {
                m_NodeHandle.getParam("/amr/hardware_interface/loop_hz", m_LoopFrequency);
            }
            else
            {
                ROS_INFO("Loop freuency is not specified in the parameter server. Defaulting back to 50 Hz");
            }
            if(m_NodeHandle.hasParam("/amr/hardware_interface/encoders/resolution"))
            {

                m_NodeHandle.getParam("/amr/hardware_interface/encoders/resolution", m_EncoderResolution);
            }
            else
            {
                ROS_INFO("Could not find encoder resolution in the parameter server. Defaulting back to 18 Bits.");
            }

            if(m_NodeHandle.hasParam("/amr/ads_config/symbols/left_encoder"))
            {
                std::string tempStr;
                m_NodeHandle.getParam("/amr/ads_config/symbols/left_encoder", tempStr);
                m_SymbolNameMap["left_encoder"] = tempStr;
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the left encoder. Shutting down...");
                ros::shutdown();
            }
            if(m_NodeHandle.hasParam("/amr/ads_config/symbols/right_encoder"))
            {
                std::string tempStr;
                m_NodeHandle.getParam("/amr/ads_config/symbols/right_encoder", tempStr);
                m_SymbolNameMap["right_encoder"] = tempStr;
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the right encoder. Shutting down...");
                ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/amr/ads_config/symbols/left_motor"))
            {
                std::string tempStr;
                m_NodeHandle.getParam("/amr/ads_config/symbols/left_motor", tempStr);
                m_SymbolNameMap["left_motor"] = tempStr;
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the left motor. Shutting down...");
                ros::shutdown();
            }
            
            if(m_NodeHandle.hasParam("/amr/ads_config/symbols/right_motor"))
            {
                std::string tempStr;
                m_NodeHandle.getParam("/amr/ads_config/symbols/right_motor", tempStr);
                m_SymbolNameMap["right_motor"] = tempStr;
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the right motor. Shutting down...");
                ros::shutdown();
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

    amr::controlled_hwi::HardwareInterface amr_hwi(nh);

    ros::waitForShutdown();

    return 0;
}