#include "../include/air_arm.hpp"

namespace air
{
    Arm::Arm(const std::vector<std::string>& joint_names)
    {
        m_JointNames = joint_names;
        m_NumOfJoints = (uint8_t)m_JointNames.size();

        m_JointPositions.resize(m_NumOfJoints);
        m_JointVelocities.resize(m_NumOfJoints);
        m_JointEfforts.resize(m_NumOfJoints);
    };

    Arm::Arm(
        const std::vector<std::string>& joint_names,
        std::shared_ptr<AdsDevice>& route_shared_ptr
    )   : m_RouteWeakPtr{route_shared_ptr}
    {
        
    }

    void Arm::init_ads_symbol_map(
        const std::vector<std::string>& names,
        const std::vector<std::string>& symbols
    )
    {
        for(std::size_t i = 0; i < names.size(); i++)
        {
            m_AdsSymbolMap[names.at(i)] = symbols.at(i);
        }
    }
}