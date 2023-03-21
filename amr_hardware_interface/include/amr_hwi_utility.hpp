#ifndef AMR_HWI_UTILITY_HPP
#define AMR_HWI_UTILITY_HPP

#include <sensor_msgs/JointState.h>
#include <ros/time.h>

namespace amr
{
    namespace utils
    {
        
        struct VelocityHelper
        {
            double velocityEncoderResolution = 0.0;
            double wheelSideGear = 0.0;
            double motorSideGear = 0.0;
            double motorGearHeat = 0.0;
            double wheelDiameter = 0.0;
            double motorMaxRPM = 0.0;
        };

        int32_t linearVelToDriverCmd(const double linear_vel, const VelocityHelper& vel_helper);

        double driverVelToLinear(const int32_t driver_vel, const VelocityHelper& vel_helper);

    }
}

#endif