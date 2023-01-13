#include "../include/amr_hwi.hpp"

namespace amr
{
           
    namespace hwi
    {
        HardwareInterface::HardwareInterface(ros::NodeHandle& nh)
            : m_NodeHandle(nh)
        {
            
            this->loadParams();
            
            m_AdsInterface = std::make_unique<io::ads_interface>(
                m_RemoteIPv4,
                m_RemoteNetIdStr,
                m_LocaleAddr,
                m_PortNum
            );

            m_AdsInterface->updateState();

            m_OdomClient = m_NodeHandle.serviceClient<amr_custom_interfaces::OdomSrv>(
                "odom_srv"
            );

            m_CmdVelSub = m_NodeHandle.subscribe(
                "cmd_vel",
                uint32_t(100),
                &HardwareInterface::callback_cmd_vel,
                this,
                ros::TransportHints().tcpNoDelay()
            );

            m_OdomPub = m_NodeHandle.advertise<nav_msgs::Odometry>("wheel_odom", 10);

            ros::Duration updateFreq = ros::Duration(1.0 / m_LoopFrequency);

            m_NonRealTimeLoop = m_NodeHandle.createTimer(
               updateFreq,
               &HardwareInterface::update,
               this
            );

        }

        void HardwareInterface::update(const ros::TimerEvent& te)
        {
            //std::lock_guard<std::mutex> guard(m_CommunicationMutex);
            m_AdsInterface->updateState();
            double left_encoder = 0.0;
            double right_encoder = 0.0;

            m_CommunicationMutex.lock();
            try
            {
                if(m_AdsInterface->getAdsState())
                {
                    std::weak_ptr<AdsDevice> routeWeakPtr = m_AdsInterface->getRoute();
                    std::shared_ptr<AdsDevice> routeTempSharedPtr = routeWeakPtr.lock();

                    AdsVariable<double> adsLeftEncoder{*routeTempSharedPtr, ""};
                    AdsVariable<double> adsRightEncoder{*routeTempSharedPtr, ""};

                    left_encoder = adsLeftEncoder;
                    right_encoder = adsRightEncoder;
                }
                
            }
            catch(const AdsException& ex)
            {
                std::cout << ex.what() << std::endl;
            }
            catch(const std::system_error& ex)
            {
                std::cout << ex.what() << std::endl;
            }

            geometry_msgs::Point encoders_lr;
            encoders_lr.x = left_encoder;
            encoders_lr.y = right_encoder;
 
            m_CommunicationMutex.unlock();

            ROS_INFO("x: %f ; y: %f", encoders_lr.x, encoders_lr.y);

            amr_custom_interfaces::OdomSrv odom_srv;
            odom_srv.request.encoders = encoders_lr;
            
            if(m_OdomClient.call(odom_srv))
            {
                // Publish Odom
                m_OdomPub.publish(odom_srv.response.odom);
            }
            else
            {
                ROS_WARN("Could not calculate odometry");
            }

        }

        void HardwareInterface::callback_cmd_vel(const geometry_msgs::TwistConstPtr& twist_msg)
        {
            //std::lock_guard<std::mutex> guard(m_CommunicationMutex);
            m_AdsInterface->updateState();
            double angular_z = twist_msg->angular.z;
            double linear_x = twist_msg->linear.x;

            m_CommunicationMutex.lock();
            try
            {
                if(m_AdsInterface->getAdsState())
                {
                    std::weak_ptr<AdsDevice> routeWeakPtr = m_AdsInterface->getRoute();
                    std::shared_ptr<AdsDevice> routeTempSharedPtr = routeWeakPtr.lock();

                    AdsVariable<double> adsLinX{*routeTempSharedPtr, ""};
                    AdsVariable<double> adsAngZ{*routeTempSharedPtr, ""};

                    adsLinX = linear_x;
                    adsAngZ = angular_z;
                }
                
            }
            catch(const AdsException& ex)
            {
                std::cout << ex.what() << std::endl;
            }
            catch(const std::system_error& ex)
            {
                std::cout << ex.what() << std::endl;
            }

            m_CommunicationMutex.unlock();
            ROS_INFO("Linear x: %f, Angular z: %f", linear_x, angular_z);            
        }

        void HardwareInterface::loadParams()
        {
            if(m_NodeHandle.hasParam("/amr/ads_config/remote_ipv4"))
            {
                m_NodeHandle.getParam("/amr/ads_config/remote_ipv4", m_RemoteIPv4);
            }
            else
            {
                ROS_ERROR("Can't find the Remote IpV4 Addres in the parameter server.");
                ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/amr/ads_config/remote_net_id"))
            {
                m_NodeHandle.getParam("/amr/ads_config/remote_net_id", m_RemoteNetIdStr);
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

                m_LocaleAddr = arr;
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
                m_PortNum = (uint16_t)tempPort;
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
        }

    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amr_hwi_node");
    ros::NodeHandle nh;
    amr::hwi::HardwareInterface hwi(nh);

    ros::spin();

    ros::waitForShutdown();

    return 0;
}