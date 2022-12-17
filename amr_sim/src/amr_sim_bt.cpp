#include "../include/amr_sim_bt.hpp"

MoveBase::MoveBase(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
{
    
    m_MoveBaseClient = std::make_unique<MoveBaseClient>("move_base", true);

}

BT::NodeStatus MoveBase::onStart()
{
    //m_SpinThread = boost::thread(&MoveBase::m_SpinThread);

    ROS_INFO("Goal Arrived.");

    while(!m_MoveBaseClient->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Can't find action server 'Move Base'.");

        return BT::NodeStatus::FAILURE;
    }

    PositionGoal btStruct;

    if(!getInput<PositionGoal>("move_base_goal", btStruct))
    {
        return BT::NodeStatus::FAILURE;
    }

    m_Goal = btStructToRosMsg(btStruct);

    m_MoveBaseClient->sendGoal(m_Goal);
    
    ROS_INFO("Sending goal.");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBase::onRunning()
{

    std::thread waiting([this](){
        this->m_MoveBaseClient->waitForResult();
    });

    waiting.detach();

    if(m_MoveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else if(m_MoveBaseClient->getState() == actionlib::SimpleClientGoalState::ACTIVE)
    {   
        m_SpinThread.join();
        return BT::NodeStatus::RUNNING;
    }
    else if(m_MoveBaseClient->getState() == actionlib::SimpleClientGoalState::ABORTED || m_MoveBaseClient->getState() == actionlib::SimpleClientGoalState::REJECTED)
    {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void MoveBase::onHalted()
{
    m_MoveBaseClient->cancelAllGoals();
}

void MoveBase::spinThread()
{   
    ROS_INFO("Spinning.");
    ros::spin();
}

move_base_msgs::MoveBaseGoal MoveBase::btStructToRosMsg(const PositionGoal& btStruct)
{
    move_base_msgs::MoveBaseGoal rosMsg;

    rosMsg.target_pose.header.frame_id = btStruct.target_frame;
    rosMsg.target_pose.header.stamp = ros::Time::now();

    rosMsg.target_pose.pose.position.x = btStruct.pos_x;
    rosMsg.target_pose.pose.position.y = btStruct.pos_y;
    rosMsg.target_pose.pose.position.z = btStruct.pos_z;

    rosMsg.target_pose.pose.orientation.x = btStruct.orient_x;
    rosMsg.target_pose.pose.orientation.y = btStruct.orient_y;
    rosMsg.target_pose.pose.orientation.z = btStruct.orient_z;
    rosMsg.target_pose.pose.orientation.w = btStruct.orient_w;

    return rosMsg;
}

/*



*/

CheckGoal::CheckGoal(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
{   
    using namespace std::placeholders;
    nh = ros::NodeHandle();

    m_MoveGoalServer = nh.advertiseService(
        "bt_move_base_goal", 
        &CheckGoal::move_goal_server_callback, this);

}

BT::NodeStatus CheckGoal::tick()
{

    while(!m_GoalArrivedToServer)
    {
        ros::spinOnce();
    }

    if(m_GoalArrivedToServer)
    {
        ROS_INFO("Goal arrived at the server.");
        setOutput<PositionGoal>("move_base_goal", m_Goal);

        m_Goal = PositionGoal();
        m_GoalArrivedToServer = false;
        return BT::NodeStatus::SUCCESS;
    }
    
    ROS_INFO("No goal arrived at the server.");
    m_GoalArrivedToServer = false;
    return BT::NodeStatus::FAILURE;
}


PositionGoal CheckGoal::rosMsgToBtStruct(const amr_custom_interfaces::AmrMoveGoalMsg& ros_msg)
{
    PositionGoal bt_port;

    bt_port.pos_x = ros_msg.pos_x;
    bt_port.pos_y = ros_msg.pos_y;
    bt_port.pos_z = ros_msg.pos_z;

    bt_port.orient_x = ros_msg.orient_x;
    bt_port.orient_y = ros_msg.orient_y;
    bt_port.orient_z = ros_msg.orient_z;
    bt_port.orient_w = ros_msg.orient_w;

    bt_port.target_frame = ros_msg.target_frame;

    return bt_port;

}


bool CheckGoal::move_goal_server_callback(amr_custom_interfaces::AmrMoveGoalSrvRequest& request, amr_custom_interfaces::AmrMoveGoalSrvResponse& response)
{
    ROS_INFO("Goal Received.");
    m_Goal = rosMsgToBtStruct(request.goal);
    m_GoalArrivedToServer = true;
    ROS_INFO("Goal Received.");
    return true;
}

/*



*/

BT::NodeStatus isBatteryOk()
{
    //std::cout << "Battery is OK." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus isBatteryFull()
{
    return BT::NodeStatus::SUCCESS;
}

/*



*/

MoveToChargingPort::MoveToChargingPort(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
{
    m_MoveBaseClient = std::make_unique<MoveBaseClient>("move_base", true);

    m_Goal.target_pose.header.frame_id = "map";
    m_Goal.target_pose.header.stamp = ros::Time::now();

    m_Goal.target_pose.pose.position.x = 3.5;
    m_Goal.target_pose.pose.orientation.w = 1.0;

}

BT::NodeStatus MoveToChargingPort::tick()
{
    while(!m_MoveBaseClient->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Can't find action server 'Move Base'.");

        return BT::NodeStatus::FAILURE;
    }

    m_MoveBaseClient->sendGoal(m_Goal);

    m_MoveBaseClient->waitForResult();

    if(m_MoveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;

}

/*
*
*
*
*/

ChargeAction::ChargeAction(const std::string& name, const BT::NodeConfiguration& config)
    : StatefulActionNode(name, config)
{

}

BT::NodeStatus ChargeAction::onStart()
{
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ChargeAction::onRunning()
{
    
    std::cout << "Charging..." << "Currently at: " << dummy*10 << "%." << std::endl;    
    if(dummy == 10){
        dummy = 0;
        return BT::NodeStatus::SUCCESS;
    }
    dummy += 1;
    return BT::NodeStatus::RUNNING;
    
}

void ChargeAction::onHalted()
{

}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "move_base_deneme");

    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<MoveBase>("moveBase");
    factory.registerSimpleCondition("isBatteryOk", std::bind(&isBatteryOk));
    factory.registerNodeType<CheckGoal>("checkGoal");

    factory.registerNodeType<MoveToChargingPort>("moveToChargingPort");
    factory.registerSimpleCondition("isBatteryFull", std::bind(&isBatteryFull));
    factory.registerNodeType<ChargeAction>("charge");

    std::string user = std::getenv("USER");

    auto tree = factory.createTreeFromFile(
        "/home/" + user + "/catkin_ws/src/amr_sim/bt_trees/sim_bt_two.xml"
    );

    while(true)
    {
        tree.tickRoot();
    }
    
    ros::shutdown();

    return 0;

}