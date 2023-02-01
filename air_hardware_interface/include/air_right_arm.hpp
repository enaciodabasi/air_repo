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
    namespace kinematics
    {

        typedef std::pair<double, double> DoublePair;

        class RightArm
        {    
            public:

            static double toRadian(double degree);
            static double toDegree(double radian);

            static double elbowRadianToLinearPosition(double position_in_radian);
            static double elbowRadianToLinearVelocity(double velocity_in_radian);
            static double elbowLinearToRadianPosition(double position_in_mm);
            static double elbowLinearToRadianVelocity(double velocity_in_mm);

            static DoublePair wristEulerToLinearPosition(DoublePair desired_pos);
            static DoublePair wristEulerToLinearVelocity(DoublePair desired_pos, DoublePair desired_vel);
            static DoublePair wristLinearToRadianPosition(DoublePair roll_pitch_in_mm);
            static DoublePair wristLinearToRadianVelocity(DoublePair roll_pitch_vel_in_mm);

            private:

            // Arm Constants

            static const double X1  {-28.11};
            static const double Y1  { 28.46};
            static const double X2  {-28.46};
            static const double Y2  {-28.11};
            static const double Z   { 22.5};
            static const double Zz  { 184.5};
        };
    }

    class RightArm : public Arm
    {
        public:

        friend class hwi::AirHardwareInterface;

        RightArm(const std::vector<std::string>& joint_names);

        RightArm(
            const std::vector<std::string>& joint_names,
            std::shared_ptr<AdsDevice>& route_shared_ptr
        );

        private:
        
        std::vector<double> m_PosCmds;

        std::vector<double> m_VelCmds;

        void write(const std::vector<double>& pos, const std::vector<double>& vel) override;

        sensor_msgs::JointState read() override;

    };

}

#endif // AIR_RIGHT_ARM_HPP
