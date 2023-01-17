#ifndef AMR_HWI_HPP
#define AMR_HWI_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/SetBool.h>
#include "amr_custom_interfaces/OdomSrv.h"

#include "AdsDevice.h"
#include "AdsLib.h"
#include "AdsVariable.h"

#include <memory>
#include <unordered_map>
#include <mutex>

#include "io_ads.hpp"

namespace amr
{

    
    namespace hwi
    {

        class HardwareInterface
        {
            public:

            HardwareInterface(ros::NodeHandle& nh);

            private:

            std::mutex m_CommunicationMutex;

            ros::NodeHandle m_NodeHandle;

            ros::ServiceServer m_DriveStatusServer;

            ros::ServiceClient m_OdomClient;
            
            // Subscribes to the cmd_vel topic
            // Message type: Twist
            ros::Subscriber m_CmdVelSub;

            ros::Publisher m_OdomPub;
            
            ros::Timer m_NonRealTimeLoop;

            std::unique_ptr<io::ads_interface> m_AdsInterface;

            std::unordered_map<std::string, std::string> m_SymbolNameMap;

            std::string m_RemoteIPv4;

            std::string m_RemoteNetIdStr;

            std::array<uint8_t, 6> m_LocaleAddr;

            uint16_t m_PortNum;
            
            double m_LoopFrequency = 50.0;

            void write(ros::Time& time);

            void read(ros::Time& time);

            void update(const ros::TimerEvent& te);

            void callback_cmd_vel(const geometry_msgs::TwistConstPtr& twist_msg);

            bool callback_drive_status_change(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

            void loadParams();

        };
    }
    
}

#endif