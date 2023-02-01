/**
 * @file air_left_arm.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/air_left_arm.hpp"

namespace air
{
    LeftArm::LeftArm(const std::vector<std::string>& joint_names)
        : Arm(joint_names)
    {

    }

    LeftArm::LeftArm(
        const std::vector<std::string>& joint_names,
        std::shared_ptr<AdsDevice>& route_shared_ptr
    )   : Arm(joint_names, route_shared_ptr)
    {
            
    }

    void LeftArm::write(const std::vector<double>& pos, const std::vector<double>& vel)
    {

    }   

    sensor_msgs::JointState LeftArm::read()
    {
        
    } 
}