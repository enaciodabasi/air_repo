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
#include <memory>

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>

#include "ads_interface.hpp"

namespace air
{
    class Arm
    {
        public:

        Arm(const std::vector<std::string>& joint_names);

        Arm(
            const std::vector<std::string>& joint_names,
            std::shared_ptr<AdsDevice>& route_shared_ptr
        );

        virtual ~Arm();

        inline void setAdsDeviceWeakPtr(std::shared_ptr<AdsDevice>& route_shared_ptr)
        {
            m_RouteWeakPtr = route_shared_ptr;
        }

        protected:
        
        std::vector<std::string> m_JointNames;

        std::weak_ptr<AdsDevice> m_RouteWeakPtr;

        uint8_t m_NumOfJoints;

        std::vector<double> m_JointPositions;

        std::vector<double> m_JointVelocities;

        std::vector<double> m_JointEfforts;        

        virtual void write() = 0;

        virtual void read() = 0;

    };

}

#endif // AIR_ARM_HPP