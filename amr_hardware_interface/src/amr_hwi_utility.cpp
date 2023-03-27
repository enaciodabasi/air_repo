#include "../include/amr_hwi_utility.hpp"

namespace amr
{
    namespace utils
    {
        double driverVelToLinear(int32_t driver_vel, const VelocityHelper& vel_helper)
        {

            double linearVel = ( driver_vel * M_PI * vel_helper.wheelDiameter * vel_helper.motorSideGear ) / 
            (vel_helper.wheelSideGear * vel_helper.motorGearHeat * vel_helper.velocityEncoderResolution );

            return linearVel;
        }

        int32_t linearVelToDriverCmd(const double linear_vel, const VelocityHelper& vel_helper)
        {
            int32_t driverCmd = ( linear_vel * vel_helper.wheelSideGear * vel_helper.motorGearHeat * vel_helper.velocityEncoderResolution ) /
            ( M_PI * vel_helper.wheelDiameter * vel_helper.motorSideGear );

            return driverCmd;
        }
    }
}