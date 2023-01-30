/**
 * @file air_right_arm.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/air_right_arm.hpp"

namespace air
{
    RightArm::RightArm(const std::vector<std::string>& joint_names)
        : Arm(joint_names)
    {
        
    }

    RightArm::RightArm(
        const std::vector<std::string>& joint_names,
        std::shared_ptr<AdsDevice>& route_shared_ptr
    )   : Arm(joint_names, route_shared_ptr)
    {
            
    }

    void RightArm::write()
    {

    }   

    void RightArm::read()
    {

    } 
}