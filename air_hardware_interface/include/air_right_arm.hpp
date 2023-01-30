/**
 * @file air_right_arm.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef AIR_RIGHT_ARM_HPP
#define AIR_RIGHT_ARM_HPP

#include <ros/ros.h>

#include "air_arm.hpp"
#include "air_hardware_interface.hpp"

namespace air
{
    class RightArm : public Arm
    {
        public:

        friend class hwi::AirHardwareInterface;

        RightArm(const std::vector<std::string>& joint_names);

        private:

        std::vector<double> m_PosCmds;
        std::vector<double> m_VelCmds;

    };
}

#endif // AIR_RIGHT_ARM_HPP
