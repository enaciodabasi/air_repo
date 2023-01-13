#include <ros/ros.h>
#include <amr_custom_interfaces/AmrMoveGoalSrv.h>
#include <amr_custom_interfaces/AmrMoveGoalMsg.h>
#include <iostream>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "amr_send_goal_srv");

    if(argc != 3)
    {
        ROS_INFO("Not enough position arguments.");
        return 1;
    }    

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);

    while(!client.waitForServer(ros::Duration(2.0)))
    {
        ROS_INFO("Waiting");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.pose.position.x = atof(argv[1]);
    goal.target_pose.pose.orientation.w = atof(argv[2]);

    client.sendGoal(goal);

    client.waitForResult(); 

    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Moved.");
    }

    //ros::NodeHandle nh;
    //ros::ServiceClient client = nh.serviceClient<amr_custom_interfaces::AmrMoveGoalSrv>("bt_move_base_goal");
//
    //amr_custom_interfaces::AmrMoveGoalSrv srv;
    //srv.request.goal.pos_x = atof(argv[1]);
    //srv.request.goal.orient_w = atof(argv[2]);
    //srv.request.goal.target_frame = std::string(argv[3]);
//
    //std::cout << "Sending goal to service: " << atof(argv[1]) << " " << " " << atof(argv[2]) << " " << std::string(argv[3]); 
//
    //if(client.call(srv))
    //{
    //    ROS_INFO("Goal sent.");
    //    
    //}
    //else
    //{
    //    ROS_ERROR("Failed to connect to server.");
    //    return 1;
    //}

    return 0;
}