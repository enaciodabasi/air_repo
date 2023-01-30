/**
 * @file air_left_arm.cpp
 * @author your name (you@domain.com)
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
}