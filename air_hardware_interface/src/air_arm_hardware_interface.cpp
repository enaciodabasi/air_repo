#include "../include/air_arm_hardware_interface.hpp"

#include <ostream>

namespace air
{
    RightArmHardwareInterface::RightArmHardwareInterface(ros::NodeHandle& nh) : m_NodeHandle(nh)
    {
        this->init();

        m_ControllerManager.reset(new controller_manager::ControllerManager(this, m_NodeHandle));

        ros::Duration updateFrequency = ros::Duration(1.0 / m_LoopFrequency);
        
        m_NodeHandle.param("/arm/hardware_interface/loop_hz", m_LoopFrequency, 0.1);

        m_NonRealtimeLoop = m_NodeHandle.createTimer(updateFrequency, &RightArmHardwareInterface::update, this);

    }

    RightArmHardwareInterface::~RightArmHardwareInterface()
    {

    }   

    void RightArmHardwareInterface::init()
    {
        m_NodeHandle.getParam("/arm/hardware_interface/joints", m_JointNames);
        if(m_JointNames.size() == 0)
        {
            ROS_FATAL_STREAM_NAMED("init", "No joint names found in the parameter server.");
        }

        m_NumJoints = m_JointNames.size();

        m_JointPositions.resize(m_NumJoints);
        m_JointVelocities.resize(m_NumJoints);
        m_JointEfforts.resize(m_NumJoints);
        m_PositionCommand.resize(m_NumJoints);
        m_VelocityCommand.resize(m_NumJoints);
        m_AccelCommand.resize(m_NumJoints);

        for(int i = 0; i < m_NumJoints; i++)
        {
            hardware_interface::JointStateHandle jointStateHandle(
                m_JointNames.at(i),
                &m_JointPositions.at(i),
                &m_JointVelocities.at(i),
                &m_JointEfforts.at(i)
            );

            m_JointStateInterface.registerHandle(jointStateHandle);

            hardware_interface::PosVelAccJointHandle posVelAccJointHandle(
                jointStateHandle,
                &m_PositionCommand.at(i),
                &m_VelocityCommand.at(i),
                &m_AccelCommand.at(i)
            );

            m_PosVelAccJointInterface.registerHandle(posVelAccJointHandle);
        }

        this->registerInterface(&m_JointStateInterface);
        this->registerInterface(&m_PosVelAccJointInterface);
    }

    void RightArmHardwareInterface::update(const ros::TimerEvent& te)
    {
        m_ElapsedTime = ros::Duration(te.current_real - te.last_real);

        this->read();
        m_ControllerManager->update(te.current_real, m_ElapsedTime);
        this->write(m_ElapsedTime);

        m_LogInfo = "\n";
        m_LogInfo += "Joint Position Error:\n";

        for(int i = 0; i < m_NumJoints; i++)
        {
            std::ostringstream jointPosStr;
            jointPosStr << m_PositionCommand.at(i) - m_JointPositions[i];
            m_LogInfo += "  " + m_JointNames.at(i) + ": " + jointPosStr.str() + "\n";
        }
    }

    void RightArmHardwareInterface::read()
    {

        std::array<double, 7> rawJointPositions;
    
        sensor_msgs::JointState actualJointStates = m_RightArm.readJoint(m_NumJoints, rawJointPositions);

        for(int i = 0; i < m_NumJoints; i++)
        {
            m_JointPositions.at(i) = actualJointStates.position.at(i);
            m_JointVelocities.at(i) = actualJointStates.velocity.at(i);

            // Write to log

        }

    }

    void RightArmHardwareInterface::write(const ros::Duration& elapsed_time)
    {
        m_RightArm.writeJoint(
            m_NumJoints, 
            m_JointPositions,
            m_PositionCommand,
            m_VelocityCommand,
            m_AccelCommand);
    }


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "air_right_arm_hardware_interface_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    air::RightArmHardwareInterface rightArmInterface(nh);

    ros::waitForShutdown();

    return 0;      
    
}
