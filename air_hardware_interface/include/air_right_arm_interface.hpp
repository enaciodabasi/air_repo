#ifndef AIR_RIGHT_ARM_INTERFACE_HPP
#define AIR_RIGHT_ARM_INTERFACE_HPP

#include "AdsLib.h"
#include "AdsVariable.h"

#include "ros/ros.h"

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <math.h>

#include <sensor_msgs/JointState.h>

namespace air
{

    typedef std::pair<double, double> DoublePair;

    class RightArmIK
    {   
        public:

        RightArmIK();

        static double toRadian(double degree);
        static double toDegree(double radian);

        double elbowRadianToLinearPosition(double position_in_radian);
        double elbowRadianToLinearVelocity(double position_in_radian, double velocity_in_radian);
        double elbowLinearToRadianPosition(double position_in_mm);
        double elbowLinearToRadianVelocity(double velocity_in_mm);

        DoublePair wristEulerToLinearPosition(DoublePair desired_pos);
        DoublePair wristEulerToLinearVelocity(DoublePair desired_pos, DoublePair desired_vel);
        DoublePair wristLinearToRadianPosition(DoublePair roll_pitch_in_mm);
        DoublePair wristLinearToRadianVelocity(DoublePair roll_pitch_vel_in_mm);

        private:
        
        // Arm Constants

        double X1  {-28.11};
        double Y1  { 28.46};
        double X2  {-28.46};
        double Y2  {-28.11};
        double Z   { 22.5};
        double Zz  { 184.5};

    };

    class RightArm
    {
        public:

        RightArm();

        RightArmIK m_RightArmIK;

        AdsVariable<uint32_t>* m_AdsActTimestamp;

        void setJointNames(const std::vector<std::string> joint_names);

        sensor_msgs::JointState readJoint(int num_joints, std::array<double, 7>& raw_joint_positions);

        bool writeJoint(int num_joints, std::vector<double> act_pose, std::vector<double> pose, std::vector<double> vel, std::vector<double> acc);

        double calcDeltaT(double actual_pose, double desired_pose, double desired_velocity);
        
        private:

        std::vector<std::string> m_JointNames;
        std::vector<double> m_JointPositions;

        AmsNetId m_RemoteNetID {192, 168, 6, 29, 1, 1};

        const std::string m_RemoteIpV4 = "192.168.6.29";

        AdsDevice* m_Route;

    };
}

#endif