#ifndef AMR_HWI_HPP
#define AMR_HWI_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include "amr_custom_interfaces/OdomSrv.h"

#include "AdsDevice.h"
#include "AdsLib.h"
#include "AdsVariable.h"

#include <memory>
#include <mutex>

namespace amr
{

    namespace io
    {
        class ads_interface
        {
            public:

            ads_interface(
                const std::string& remote_ipv4,
                const std::string& remote_netId,
                const std::array<uint8_t, 6>& locale,
                const uint16_t port
            );

            ~ads_interface(); 

            std::weak_ptr<AdsDevice> getRoute()
            {
                return m_Route;
            }

            bool getAdsState()
            {
                return m_AdsState;
            }

            void updateState()
            {
                checkConnection();
            }

            private:

            AmsNetId* m_RemoteNetId = nullptr;

            std::shared_ptr<AdsDevice> m_Route;

            std::string m_RemoteIPv4;

            std::string m_RemoteNetIdStr;

            std::array<uint8_t, 6> m_LocaleAddr;

            uint16_t m_PortNum;
            
            // True if state is ADSSTATE_RUN
            // Else false
            bool m_AdsState;

            void checkConnection();
        };
    }
    
    namespace hwi
    {

        class HardwareInterface
        {
            public:

            HardwareInterface(ros::NodeHandle& nh);

            private:

            std::mutex m_CommunicationMutex;

            ros::NodeHandle m_NodeHandle;

            ros::ServiceClient m_OdomClient;
            
            // Subscribes to the cmd_vel topic
            // Message type: Twist
            ros::Subscriber m_CmdVelSub;

            ros::Publisher m_OdomPub;
            
            ros::Timer m_NonRealTimeLoop;

            std::unique_ptr<io::ads_interface> m_AdsInterface;

            std::string m_RemoteIPv4;

            std::string m_RemoteNetIdStr;

            std::array<uint8_t, 6> m_LocaleAddr;

            uint16_t m_PortNum;
            
            double m_LoopFrequency = 50.0;

            void write(ros::Time& time);

            void read(ros::Time& time);

            void update(const ros::TimerEvent& te);

            void callback_cmd_vel(const geometry_msgs::TwistConstPtr& twist_msg);

            void loadParams();

        };
        }
    
}

#endif