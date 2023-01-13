#ifndef IO_ADS_HPP
#define IO_ADS_HPP

#include <iostream>
#include <memory>

#include "AdsLib.h"
#include "AdsDevice.h"
#include "AdsVariable.h"

namespace amr
{
    namespace io
    {

        struct AdsInfo
        {
            std::string remote_ipv4;
            std::string remote_netId;
            std::array<uint8_t, 6> local_address;
            uint16_t port_num;
        };

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
}


#endif