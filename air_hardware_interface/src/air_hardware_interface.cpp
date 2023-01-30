/**
 * @file air_hardware_interface.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "../include/air_hardware_interface.hpp"

namespace air
{
    namespace hwi
    {
        AirHardwareInterface::AirHardwareInterface(ros::NodeHandle& nh)
        {
            RobotParams robotParams = loadRobotParams();

            m_RightArm = new RightArm(robotParams.jointNamesRightArm);
            m_LeftArm = new LeftArm(robotParams.jointNamesLeftArm);

            m_TotalNumberOfJoints = (int)m_RightArm->m_NumOfJoints + (int)m_LeftArm->m_NumOfJoints;

            for(int i = 0; i < m_RightArm->m_NumOfJoints; i++)
            {
                hardware_interface::JointStateHandle jointStateHandle(
                    m_RightArm->m_JointNames[i],
                    &m_RightArm->m_JointPositions[i],
                    &m_RightArm->m_JointVelocities[i],
                    &m_RightArm->m_JointEfforts[i]
                );

                m_JointStateInterface.registerHandle(jointStateHandle);

                hardware_interface::PosVelJointHandle posVelJointHandle(
                    jointStateHandle,
                    &m_RightArm->m_PosCmds[i],
                    &m_RightArm->m_VelCmds[i]
                );

                m_PosVelJointInterface.registerHandle(posVelJointHandle);
            }

            int j = 0;
            while(j < m_LeftArm->m_NumOfJoints)
            {
                hardware_interface::JointStateHandle jointStateHandle(
                    m_LeftArm->m_JointNames[j],
                    &m_LeftArm->m_JointPositions[j],
                    &m_LeftArm->m_JointVelocities[j],
                    &m_LeftArm->m_JointEfforts[j]
                );

                m_JointStateInterface.registerHandle(jointStateHandle);

                hardware_interface::PosVelJointHandle posVelJointHandle(
                    jointStateHandle,
                    &m_LeftArm->m_PosCmds[j],
                    &m_LeftArm->m_VelCmds[j]
                );

                m_PosVelJointInterface.registerHandle(posVelJointHandle);

                j += 1;
            }

            this->registerInterface(&m_JointStateInterface);
            this->registerInterface(&m_PosVelJointInterface);

        }


        RobotParams AirHardwareInterface::loadRobotParams()
        {   
            RobotParams robotParams;

            if(m_NodeHandle.hasParam("/air_arm/joints/right_arm"))
            {
                m_NodeHandle.getParam("/air_arm/joints/right_arm", robotParams.jointNamesRightArm);
            }
            else
            {
                ROS_ERROR("No right arm joint names find in the parameter server. Shutting down the hardware interface.");
                if(ros::ok())
                    ros::shutdown();
            }

            if(m_NodeHandle.hasParam("/air_arm/joints/left_arm"))
            {
                m_NodeHandle.getParam("/air_arm/joints/left_arm", robotParams.jointNamesLeftArm);
            }
            else
            {
                ROS_ERROR("No left arm joint names find in the parameter server. Shutting down the hardware interface.");
                if(ros::ok())
                    ros::shutdown();
            }

            return robotParams;
        }
    }
}