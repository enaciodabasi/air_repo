#ifndef AMR_SIM_BT_HPP
#define AMR_SIM_BT_HPP

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <amr_custom_interfaces/AmrMoveGoalSrv.h>
#include <amr_custom_interfaces/AmrMoveGoalMsg.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <iostream>
#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <functional>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>

struct PositionGoal
{
    double pos_x;
    double pos_y;
    double pos_z;

    double orient_x;
    double orient_y;
    double orient_z;
    double orient_w;

    std::string target_frame;

};

namespace BT
{
    template<> inline PositionGoal convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');

        if(parts.size() != 8)
        {
            throw RuntimeError("invalid input");
        }
        else
        {
            PositionGoal output;
            output.pos_x = convertFromString<double>(parts[0]);
            output.pos_y = convertFromString<double>(parts[1]);
            output.pos_z = convertFromString<double>(parts[2]);
            output.orient_x = convertFromString<double>(parts[3]);
            output.orient_y = convertFromString<double>(parts[4]);
            output.orient_z = convertFromString<double>(parts[5]);
            output.orient_w = convertFromString<double>(parts[6]);
            output.target_frame = convertFromString<std::string>(parts[7]);
            
            return output;
        }
    }
}

class MoveBase : public BT::StatefulActionNode
{

    public:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    MoveBase(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;
    
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<PositionGoal>("move_base_goal")};
    }

    private:

    std::unique_ptr<MoveBaseClient> m_MoveBaseClient;

    move_base_msgs::MoveBaseGoal m_Goal;
    
    std::thread m_waitThread;

    void spinThread();

    void wait_thread();

    move_base_msgs::MoveBaseGoal btStructToRosMsg(const PositionGoal& btStruct);

};

class CheckGoal : public BT::SyncActionNode
{
    public:

    CheckGoal(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<PositionGoal>("move_base_goal")};
    }

    private:
    
    ros::NodeHandle nh;

    ros::ServiceServer m_MoveGoalServer;

    std::unique_ptr<std::thread> m_RosSpinnerThread;

    bool m_GoalArrivedToServer = false;

    PositionGoal m_Goal;

    PositionGoal rosMsgToBtStruct(const amr_custom_interfaces::AmrMoveGoalMsg& ros_msg);    

    bool move_goal_server_callback(amr_custom_interfaces::AmrMoveGoalSrvRequest& request, amr_custom_interfaces::AmrMoveGoalSrvResponse& response);

    void ros_spinner_thread();

};

BT::NodeStatus isBatteryFull();

class MoveToChargingPort : public BT::SyncActionNode
{
    public:
    
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    MoveToChargingPort(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return {BT::PortsList()};
    }

    BT::NodeStatus tick() override;

    private:

    std::unique_ptr<MoveBaseClient> m_MoveBaseClient;

    move_base_msgs::MoveBaseGoal m_Goal;

};

class ChargeAction : public BT::StatefulActionNode
{
    public:

    ChargeAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return {BT::PortsList()};
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    private:

    int dummy = 0;


};

#endif