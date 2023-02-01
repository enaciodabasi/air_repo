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
#include "AdsVariable.h"
#include <cmath>

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

    void RightArm::write(const std::vector<double>& pos, const std::vector<double>& vel)
    {

        std::array<double, 7> posToWrite;
        std::array<double, 7> velToWrite;

        // s1, s2, s3
            for(int i = 0; i < 3; i++)
            {
                posToWrite[i] = kinematics::RightArm::toDegree(pos.at(i));
                velToWrite[i] = kinematics::RightArm::toDegree(vel.at(i));
            }

            // elbow

            double beckhoff_urdf_error = M_PI / 2.0;
            double elbow_desired_position = pos.at(3);
            elbow_desired_position += beckhoff_urdf_error;

            posToWrite[3] = kinematics::RightArm::elbowRadianToLinearPosition(elbow_desired_position);
            velToWrite[3] = kinematics::RightArm::elbowRadianToLinearVelocity(vel.at(3));
            // Roll | Pitch Joints

            std::pair<double, double> desired_rp_angular_position;
            std::pair<double, double> desired_rp_angular_velocity;
            std::pair<double, double> desired_rp_linear_position;
            std::pair<double, double> desired_rp_linear_vel;

            desired_rp_angular_position = std::make_pair(pos.at(4), pos.at(5));
            desired_rp_angular_velocity = std::make_pair(vel.at(4), vel.at(5));
            desired_rp_linear_position = kinematics::RightArm::wristEulerToLinearPosition(desired_rp_angular_position);
            desired_rp_linear_vel = kinematics::RightArm::wristEulerToLinearVelocity(desired_rp_angular_position, desired_rp_angular_velocity);
            
            posToWrite[4] = desired_rp_linear_position.first;
            velToWrite[4] = desired_rp_linear_vel.first;

            posToWrite[5] = desired_rp_linear_position.second;
            velToWrite[5] = desired_rp_linear_vel.second;

            // Yaw Joint

            posToWrite[6] = kinematics::RightArm::toDegree(pos.at(6));
            velToWrite[6] = kinematics::RightArm::toDegree(vel.at(6));

            try
            {
                auto tempSharedRoute = m_RouteWeakPtr.lock();

                AdsVariable<std::array<double, 7>> positionToAds{*tempSharedRoute, m_AdsSymbolMap.find("goal_pos")->second};
                AdsVariable<std::array<double, 7>> velocityToAds{*tempSharedRoute, m_AdsSymbolMap.find("goal_vel")->second};

                positionToAds = posToWrite;
                velocityToAds = velToWrite;
            }
            catch(const AdsException& ex)
            {
                std::cout << "Error Code: " << ex.errorCode << std::endl;
                std::cout << "AdsException message: " << ex.what() << std::endl;
            }catch(const std::runtime_error& ex)
            {
                std::cout << ex.what() << std::endl;
            }
    }   

    sensor_msgs::JointState RightArm::read()
    {

        sensor_msgs::JointState currentJointState;
        std::array<double, 7> positionArray;
        std::array<double, 7> velocityArray;

        try{

            auto tempSharedRoute = m_RouteWeakPtr.lock();

            AdsVariable<std::array<double, 7>> positionAds{*tempSharedRoute, m_AdsSymbolMap.find("act_pos")->second};
            AdsVariable<std::array<double, 7>> velocityAds{*tempSharedRoute, m_AdsSymbolMap.find("act_vel")->second};

            positionArray = positionAds;
            velocityArray = velocityAds;

        }
        catch(const AdsException& ex){

            std::cout << "Error Code: " << ex.errorCode << std::endl;
            std::cout << "AdsException message: " << ex.what() << std::endl;

        }catch(const std::runtime_error& ex){

            std::cout << ex.what() << std::endl;
        }

        // s1, s2, s3 joints:

        for(int i = 0; i < 3; i++)
        {
            currentJointState.position.push_back(kinematics::RightArm::toRadian(positionArray[i]));
            currentJointState.velocity.push_back(kinematics::RightArm::toRadian(velocityArray[i]));
        }

        // Elbow joint:

        currentJointState.position.push_back(
            kinematics::RightArm::elbowLinearToRadianPosition(positionArray[3])
        );
        currentJointState.velocity.push_back(
            kinematics::RightArm::elbowLinearToRadianVelocity(velocityArray[3])
        );

        // Roll and Pitch joints:

        std::pair<double, double> actual_RP_Euler_Position = kinematics::RightArm::wristLinearToRadianPosition(std::pair<double, double>(positionArray[4], positionArray[5]));
        std::pair<double, double> actual_RP_Euler_Velocity = kinematics::RightArm::wristLinearToRadianVelocity(std::pair<double, double>(velocityArray[4], velocityArray[5])); 

        currentJointState.position.push_back(actual_RP_Euler_Position.first);
        currentJointState.velocity.push_back(actual_RP_Euler_Velocity.first);
        currentJointState.position.push_back(actual_RP_Euler_Position.second);
        currentJointState.velocity.push_back(actual_RP_Euler_Velocity.second);

        // Yaw Joint:

        currentJointState.position.push_back(kinematics::RightArm::toRadian(positionArray[6]));
        currentJointState.velocity.push_back(kinematics::RightArm::toRadian(velocityArray[6]));

        return currentJointState;
    } 

    namespace kinematics
    {
        DoublePair RightArm::wristEulerToLinearPosition(DoublePair desired_pos)
        {
            const double r = desired_pos.first;
            const double p = desired_pos.second;
    
            const double C1 = std::cos(r);
            const double C2 = std::cos(p);
            const double S1 = std::sin(r);
            const double S2 = std::sin(p);
    
            double pitch_command = 162.0 - sqrt(pow(X1-X1*C2,2) + pow((Y1-(X1*S1*S2+Y1*C1)),2) +
                                  pow((Z-(Zz-X1*C1*S2+Y1*S1)),2));
    
            double roll_command = 162.0 - sqrt(pow(X2-X2*C2,2) + pow((Y2-(X2*S1*S2+Y2*C1)),2) +
                                  pow((Z-(Zz-X2*C1*S2+Y2*S1)),2));
    
            return std::make_pair(roll_command, pitch_command);
        }

        DoublePair RightArm::wristEulerToLinearVelocity(DoublePair desired_pos, DoublePair desired_vel)
        {
            double th_roll  = desired_pos.first;
            double th_pitch = desired_pos.second;
            double th_dot_roll  = desired_vel.first;
            double th_dot_pitch = desired_vel.second;
    
            double C1 = std::cos(th_roll);
            double C2 = std::cos(th_pitch);
            double S1 = std::sin(th_roll);
            double S2 = std::sin(th_pitch);
    
            double pitchL = 162.0 - sqrt(pow(X1-X1*C2,2) + pow((Y1-(X1*S1*S2+Y1*C1)),2) +
                                  pow((Z-(Zz-X1*C1*S2+Y1*S1)),2));
            double rollL = 162.0 - sqrt(pow(X2-X2*C2,2) + pow((Y2-(X2*S1*S2+Y2*C1)),2) +
                                        pow((Z-(Zz-X2*C1*S2+Y2*S1)),2));
    
            double pitch_vel_cmd = (2*X1*X1*S2*th_dot_pitch - 2*Y1*X1*C1*S2*th_dot_roll - 2*Y1*X1*S1*C2*th_dot_pitch +
                              2*Y1*Y1*S1*th_dot_roll -2*Z*X1*S1*S2*th_dot_roll + 2*Z*X1*C1*C2*th_dot_pitch - 2*Z*Y1*C1*th_dot_roll +
                              2*Zz*X1*S1*S2*th_dot_roll - 2*Zz*X1*C1*C2*th_dot_pitch + 2*Zz*Y1*C1*th_dot_roll)/(2*(pitchL - 162.0));
    
    
            double roll_vel_cmd = (2*(X2 - X2*C2)*X2*S2*th_dot_pitch + 2*(Y2 - (X2*S1*S2 + Y2*C1))*(Y2*S1*th_dot_roll -
                            X2*C1*S2*th_dot_roll - X2*S1*C2*th_dot_pitch) + 2*((Z - Zz) +
                            (X2*C1*S2 - Y2*S1))*(-X2*S1*S2*th_dot_roll + X2*C1*C2*th_dot_pitch - Y2*C1*th_dot_roll))/(2*(rollL - 162.0));
    
    
            return std::make_pair(roll_vel_cmd, pitch_vel_cmd);
        }

        DoublePair RightArm::wristLinearToRadianPosition(DoublePair roll_pitch_in_mm)
        {
            const double rollxpiston = roll_pitch_in_mm.first;
            const double pitchypiston = roll_pitch_in_mm.second;

            const double p00 = 0.001665;
            const double p10 = 1;
            const double p01 = 1.023;
            const double p20 = 0.001848;
            const double p11 = -0.001393;
            const double p02 = 0.0004754;
            const double p30 = 3.407e-05;
            const double p21 = 0.0001189;
            const double p12 = 0.0001038;
            const double p03 = 0.000226;
            const double p40 = 5.659e-06;
            const double p31 = 4.071e-06;
            const double p22 = 6.077e-06;
            const double p13 = 2.604e-06;
            const double p04 = -2.154e-06;

            const double r00 = -8.671e-06;
            const double r10 = 1.019;
            const double r01 = -1.007;
            const double r20 = 2.939e-05;
            const double r11 = -1.657e-05;
            const double r02 = 1.088e-05;
            const double r30 = 4.714e-05;
            const double r21 = -0.0001397;
            const double r12 = 0.0001287;
            const double r03 = -3.563e-05;
            const double r40 = -9.05e-08;
            const double r31 = -4.625e-07;
            const double r22 = 2.809e-06;
            const double r13 = -3.96e-06;
            const double r04 = 1.643e-06;

            double roll_rad = toRadian(r00 + r10*rollxpiston + r01*pitchypiston + r20*pow(rollxpiston,2) + r11*rollxpiston*pitchypiston + r02*pow(pitchypiston,2) + r30*pow(rollxpiston,3) + r21*pow(rollxpiston,2)*pitchypiston + r12*rollxpiston*pow(pitchypiston,2) + r03*pow(pitchypiston,3) + r40*pow(rollxpiston,4) + r31*pow(rollxpiston,3)*pitchypiston + r22*pow(rollxpiston,2)*pow(pitchypiston,2) + r13*rollxpiston*pow(pitchypiston,3) + r04*pow(pitchypiston,4));
            double pitch_rad = -toRadian(p00 + p10*rollxpiston + p01*pitchypiston + p20*pow(rollxpiston,2) + p11*rollxpiston*pitchypiston + p02*pow(pitchypiston,2) + p30*pow(rollxpiston,3) + p21*pow(rollxpiston,2)*pitchypiston + p12*rollxpiston*pow(pitchypiston,2) + p03*pow(pitchypiston,3) + p40*pow(rollxpiston,4) + p31*pow(rollxpiston,3)*pitchypiston + p22*pow(rollxpiston,2)*pow(pitchypiston,2) + p13*rollxpiston*pow(pitchypiston,3) + p04*pow(pitchypiston,4));

            return std::make_pair(roll_rad, pitch_rad);
        }

        DoublePair RightArm::wristLinearToRadianVelocity(DoublePair roll_pitch_vel_in_mm)
        {
            return roll_pitch_vel_in_mm;
        }

        double RightArm::elbowLinearToRadianPosition(double position_in_mm)
        {
            const double D    =150.57;
            const double L    =180.50;
            const double o1   =28.0;
            const double o2   =33.0;
            const double r    =54.95;
            const double h    =226.0;
            double h1   {std::sqrt(h*h+o2*o2)};

            const double m     =position_in_mm;
            double Ly   {std::sqrt(o1*o1+(m+L)*(m+L))};
            double A    {std::acos((r*r+h1*h1-Ly*Ly)/(2.0*r*h1))};
            double B    {-360.0+90.0+D+toDegree(A+std::atan2(o2,h))};
            return      toRadian(B);
        }

        double RightArm::elbowLinearToRadianVelocity(double velocity_in_mm)
        {
            constexpr double r    {54.95};
            return      velocity_in_mm/r;
        }

        double RightArm::elbowRadianToLinearPosition(double position_in_radian)
        {
            const double D    =150.57;
            const double L    =180.50;
            const double o1   =28.0;
            const double o2   =33.0;
            const double r    =54.95;
            const double h    =226.0;
            double h1   {std::sqrt(h*h+o2*o2)};
            double A    {position_in_radian+2.0*M_PI-M_PI/2.0-toRadian(D)-std::atan2(o2,h)};
            return      sqrt((r*r+h1*h1-2.0*r*h1*cos(A))-(o1*o1))-L;
        }
        
        double RightArm::elbowRadianToLinearVelocity(double velocity_in_radian)
        {
            constexpr double r    {54.95};
            return      r*velocity_in_radian;
        }

        double RightArm::toRadian(double degree)
        {
            return (degree / 180.0) * M_PI;
        }

        double RightArm::toDegree(double radian)
        {
            return (radian / M_PI) * 180.0;
        }
    }
}