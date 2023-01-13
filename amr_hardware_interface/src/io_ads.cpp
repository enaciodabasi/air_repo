#include "../include/io_ads.hpp"

namespace amr
{
    namespace io
    {
        ads_interface::ads_interface(
            const std::string& remote_ipv4,
            const std::string& remote_netId,
            const std::array<uint8_t, 6>& locale,
            const uint16_t port
        )   : m_RemoteIPv4{remote_ipv4}, m_RemoteNetIdStr{remote_netId}, m_LocaleAddr{locale}, m_PortNum{port}
        {
            AmsNetId localAddr{
                m_LocaleAddr[0],
                m_LocaleAddr[1],
                m_LocaleAddr[2],
                m_LocaleAddr[3],
                m_LocaleAddr[4],
                m_LocaleAddr[5]
            };

            bhf::ads::SetLocalAddress(localAddr);

            m_RemoteNetId = new AmsNetId(m_RemoteNetIdStr);

            m_Route = std::make_shared<AdsDevice>(m_RemoteIPv4, *m_RemoteNetId, m_PortNum);


        }

        ads_interface::~ads_interface()
        {

            if(m_RemoteNetId)
            {
                delete m_RemoteNetId;
            }
        }

        void ads_interface::checkConnection()
        {

            AdsDeviceState test;
            try{

                test = m_Route->GetState();
                m_AdsState = (test.ads == ADSSTATE_RUN);
            }catch(...)
            {
                m_AdsState = false;
            }

        }
    }
}