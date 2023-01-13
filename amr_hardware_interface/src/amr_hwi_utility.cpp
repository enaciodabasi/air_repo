#include "../include/amr_hwi_utility.hpp"

namespace amr
{
    namespace utils
    {
        WheelState::WheelState(const std::string& wheel_name, const int encoder_resolution)
            : m_WheelName(wheel_name), m_PrevEncoderTicks{0}, m_LastUpdateTime(0, 0), m_EncoderResolution(encoder_resolution)
        {
            m_State.angularPos = 0.0;
            m_State.angularVel = 0.0;
        }

        State WheelState::calculateWheelState(const ros::Time& current_time, const long encoder_ticks)
        {
            ros::Duration deltaT = current_time - m_LastUpdateTime;
            
            double deltaT_secs = deltaT.toSec();

            double deltaTicks = encoder_ticks - m_PrevEncoderTicks;

            double deltaAngle = ticksToAngle(deltaTicks);
            
            m_State.angularPos += deltaAngle;
            m_State.angularVel += deltaAngle / deltaT_secs;

            m_LastUpdateTime = current_time;
            m_PrevEncoderTicks = encoder_ticks;

        }
    }
}