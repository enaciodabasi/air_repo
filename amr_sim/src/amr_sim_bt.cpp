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

    m_waitThread = std::thread(&MoveBase::wait_thread, this);
    m_waitThread.detach();


    
    return BT::NodeStatus::RUNNING;
}

void MoveBase::wait_thread()
{   
    
    ROS_INFO("Waiting..");
    bool x = this->m_MoveBaseClient->waitForResult();
    ROS_INFO("Result.");    
    /* if(m_MoveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {   
        ROS_INFO("Goal achieved from thread.");
    } */

}

BT::NodeStatus MoveBase::onRunning()
{   

    
    ROS_INFO("trying... ");
    
    if(m_MoveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {   
        std::cout << "Goal achieved." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else if(m_MoveBaseClient->getState() == actionlib::SimpleClientGoalState::ABORTED || m_MoveBaseClient->getState() == actionlib::SimpleClientGoalState::REJECTED)
    {   
        ROS_INFO("Aborted.");
        return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::RUNNING;
}

void MoveBase::onHalted()
{
    m_MoveBaseClient->cancelAllGoals();
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
    
    //m_RosSpinnerThread = new std::thread([this](){ros::spin();});

    //m_RosSpinnerThread = std::make_unique<std::thread>(&CheckGoal::ros_spinner_thread, this);
}

BT::NodeStatus CheckGoal::tick()
{

    std::thread t(&CheckGoal::ros_spinner_thread, this);
    t.join();
    //if(m_RosSpinnerThread == nullptr)
    //{
    //    m_RosSpinnerThread = new std::thread([this](){
    //        
    //        while(!this->m_GoalArrivedToServer)
    //        {
    //            ros::spinOnce();
    //        }
    //    });
//
    //    //m_RosSpinnerThread->detach();
    //}


    if(m_GoalArrivedToServer)
    {   
        ROS_INFO("Goal arrived at the server.");
        setOutput<PositionGoal>("move_base_goal", m_Goal);

        m_Goal = PositionGoal();
        m_GoalArrivedToServer = false;

        //if(t.joinable())
        //{
        //    t.join();
        //}

        return BT::NodeStatus::SUCCESS;
    }
    
    ROS_INFO("No goal arrived at the server.");
    m_GoalArrivedToServer = false;
    return BT::NodeStatus::FAILURE;
}

void CheckGoal::ros_spinner_thread()
{
    //int timeout = 10;
    //std::time_t t1 = std::time(0);
    //std::time_t t2 = std::time(0);
    //while((t2 - t1) >= timeout)
    //{   
    //    ROS_INFO("ooo");
    //    ros::spinOnce();
    //}
    
    auto start = std::chrono::high_resolution_clock::now();


    auto curr = std::chrono::high_resolution_clock::now();

    while(!m_GoalArrivedToServer)
    {   


        if(std::chrono::duration_cast<std::chrono::seconds>(curr - start).count() >= 3)
            break;
        ros::spinOnce();

        curr = std::chrono::high_resolution_clock::now();
    } 
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
    this->m_Goal = rosMsgToBtStruct(request.goal);
    this->m_GoalArrivedToServer = true;
    response.state = "running";
    ROS_INFO("%f", this->m_Goal.pos_x);
    
    return true;
}

/*



*/

BT::NodeStatus isBatteryOk()
{
    std::cout << "Battery is OK." << std::endl;
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