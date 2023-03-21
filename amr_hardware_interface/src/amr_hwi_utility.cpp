#include "../include/amr_hwi_utility.hpp"

namespace amr
{
    namespace utils
    {
        double driverVelToLinear(int32_t driver_vel, const VelocityHelper& vel_helper)
        {

            double linearVel = (driver_vel) *
            (60.0 / vel_helper.velocityEncoderResolution)*
            (M_PI * vel_helper.wheelDiameter / 60.0) / 
            (vel_helper.wheelSideGear / (vel_helper.motorSideGear * vel_helper.motorGearHeat));

            return linearVel;
        }

        int32_t linearVelToDriverCmd(const double linear_vel, const VelocityHelper& vel_helper)
        {
            int32_t driverCmd = (
                linear_vel * 
                vel_helper.motorSideGear *
                vel_helper.motorGearHeat *
                vel_helper.velocityEncoderResolution) / 
                (M_PI * vel_helper.wheelDiameter * vel_helper.wheelSideGear);

            return driverCmd * int32_t(vel_helper.motorMaxRPM);
        }
    }
}