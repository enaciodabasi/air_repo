/**
 * @file air_arm.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief Abstract class for controling the hardware interface of left and right arms of the AIR Robot. 
 * @version 0.1
 * @date 2023-01-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef AIR_ARM_HPP
#define AIR_ARM_HPP

#include <iostream>

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>

namespace air
{
    class Arm
    {
        public:

        Arm(const std::vector<std::string>& joint_names);

        virtual ~Arm();

        protected:
        
        std::vector<std::string> m_JointNames;

        uint8_t m_NumOfJoints;

        std::vector<double> m_JointPositions;

        std::vector<double> m_JointVelocities;

        std::vector<double> m_JointEfforts;        

    };

}

#endif // AIR_ARM_HPP