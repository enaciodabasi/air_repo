/**
 * @file air_left_arm.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef AIR_LEFT_ARM_HPP
#define AIR_LEFT_ARM_HPP


#include <ros/ros.h>

#include "air_arm.hpp"
#include "air_hardware_interface.hpp"

namespace air
{
    class LeftArm : public Arm
    {
        public:
        
        friend class hwi::AirHardwareInterface;

        LeftArm(const std::vector<std::string>& joint_names);

        LeftArm(
            const std::vector<std::string>& joint_names,
            std::shared_ptr<AdsDevice>& route_shared_ptr
        );

        private:

        std::vector<double> m_PosCmds;

        std::vector<double> m_VelCmds;

        void write() override;

        void read() override;

    };
}

#endif // AIR_LEFT_ARM_HPP