#include "../include/amr_ddc_hwi.hpp"

namespace amr
{
    namespace controlled_hwi
    {
        HardwareInterface::HardwareInterface(ros::NodeHandle& nh, bool use_ticks)
            : m_NodeHandle{nh}, m_UseTicks(use_ticks)
        {
            this->loadParams();

            if(use_ticks)
            {
                m_LeftWheelState = new utils::WheelState(
                m_JointNames.at(0),
                m_EncoderResolution
            );

                m_RightWheelState = new utils::WheelState(
                    m_JointNames.at(1),
                    m_EncoderResolution
                );
            }

            m_DriveStatusServer = m_NodeHandle.advertiseService(
                "change_drive_status",
                &HardwareInterface::callback_drive_status_change,
                this
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

                AdsVariable<double> leftMotorAds{*routeSharedPtr, m_SymbolNameMap.find("left_motor")->second};
                AdsVariable<double> rightMotorAds{*routeSharedPtr, m_SymbolNameMap.find("right_motor")->second};

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
            if(m_UseTicks)
            {
                long encoderLeft = 0;
                long encoderRight = 0;

                // Get encoder ticks from Beckhoff via ADS.
                try
                {   
                    std::weak_ptr<AdsDevice> routeWeakPtr =  m_AdsInterface->getRoute();
                    std::shared_ptr<AdsDevice> routeSharedPtr = routeWeakPtr.lock();

                    AdsVariable<long> leftEncoder_Ads{*routeSharedPtr, m_SymbolNameMap.find("left_encoder")->second};
                    AdsVariable<long> rightEncoder_Ads{*routeSharedPtr, m_SymbolNameMap.find("right_encoder")->second};

                    encoderLeft = leftEncoder_Ads;
                    encoderRight = rightEncoder_Ads;

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
            else
            {
                double leftWheelVel = 0.0;
                double leftWheelPos = 0.0;
                double rightWheelVel = 0.0;
                double rightWheelPos = 0.0;

                try
                {
                    std::weak_ptr<AdsDevice> routeWeakPtr =  m_AdsInterface->getRoute();
                    std::shared_ptr<AdsDevice> routeSharedPtr = routeWeakPtr.lock();

                    AdsVariable<double> leftWheelVel_Ads{*routeSharedPtr, m_SymbolNameMap.find("left_wheel_vel")->second};
                    AdsVariable<double> leftWheelPos_Ads{*routeSharedPtr, m_SymbolNameMap.find("left_wheel_pos")->second};
                    AdsVariable<double> rightWheelVel_Ads{*routeSharedPtr, m_SymbolNameMap.find("right_wheel_vel")->second};
                    AdsVariable<double> rightWheelPos_Ads{*routeSharedPtr, m_SymbolNameMap.find("right_wheel_pos")->second};

                    leftWheelPos = leftWheelPos_Ads;
                    leftWheelVel = leftWheelVel_Ads;

                    rightWheelPos = rightWheelPos_Ads;
                    rightWheelVel = rightWheelVel_Ads;

                }
                catch(AdsException& ex)
                {
                    std::cout << ex.what();
                }
                catch(std::system_error& ex)
                {
                    std::cout << ex.what();
                }

                m_JointPositions[0] = leftWheelPos;
                m_JointVelocities[0] = leftWheelVel;
                m_JointEfforts[0] = 0.0;

                m_JointPositions[1] = rightWheelPos;
                m_JointVelocities[1] = rightWheelVel;
                m_JointEfforts[1] = 0.0;
            }
            

        }

        bool HardwareInterface::callback_drive_status_change(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
        {

            try
            {
                if(m_AdsInterface->getAdsState())
                {
                    std::weak_ptr<AdsDevice> routeWeakPtr = m_AdsInterface->getRoute();
                    std::shared_ptr<AdsDevice> routeTempSharedPtr = routeWeakPtr.lock();

                    AdsVariable<bool> disableDrives{*routeTempSharedPtr, m_SymbolNameMap.find("drive_status")->second};

                    disableDrives = request.data;

                    response.success = true;
                }
                
            }
            catch(const AdsException& ex)
            {
                std::cout << ex.what() << std::endl;
                response.success = false;
            }
            catch(const std::system_error& ex)
            {
                std::cout << ex.what() << std::endl;
                response.success = false;
            }
            
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
            if(m_UseTicks)
            {
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
            }
            else
            {
                if(m_NodeHandle.hasParam("/amr/ads_config/symbols/wheels/left/vel"))
                {
                    std::string tempStr;
                    m_NodeHandle.getParam("/amr/ads_config/symbols/wheels/left/vel", tempStr);
                    m_SymbolNameMap["left_wheel_vel"] = tempStr;
                }
                else
                {
                    ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the left wheel velocity. Shutting down...");
                    ros::shutdown();
                }
                if(m_NodeHandle.hasParam("/amr/ads_config/symbols/wheels/left/pos"))
                {
                    std::string tempStr;
                    m_NodeHandle.getParam("/amr/ads_config/symbols/wheels/left/pos", tempStr);
                    m_SymbolNameMap["left_wheel_pos"] = tempStr;
                }
                else
                {
                    ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the left wheel position. Shutting down...");
                    ros::shutdown();
                }
                if(m_NodeHandle.hasParam("/amr/ads_config/symbols/wheels/right/vel"))
                {
                    std::string tempStr;
                    m_NodeHandle.getParam("/amr/ads_config/symbols/wheels/right/vel", tempStr);
                    m_SymbolNameMap["right_wheel_vel"] = tempStr;
                }
                else
                {
                    ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the right wheel velocity. Shutting down...");
                    ros::shutdown();
                }
                if(m_NodeHandle.hasParam("/amr/ads_config/symbols/wheels/right/pos"))
                {
                    std::string tempStr;
                    m_NodeHandle.getParam("/amr/ads_config/symbols/wheels/right/pos", tempStr);
                    m_SymbolNameMap["right_wheel_pos"] = tempStr;
                }
                else
                {
                    ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the right wheel position. Shutting down...");
                    ros::shutdown();
                }
            }
            
            if(m_NodeHandle.hasParam("/amr/ads_config/symbols/drive_status"))
            {
                std::string tempStr;
                m_NodeHandle.getParam("/amr/ads_config/symbols/drive_status", tempStr);
                m_SymbolNameMap["drive_status"] = tempStr;
            }
            else
            {
                ROS_ERROR_NAMED("ADS configuration error", "Can't find symbol name for the drive status.");
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