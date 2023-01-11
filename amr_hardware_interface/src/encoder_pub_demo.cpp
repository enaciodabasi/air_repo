#include <ros/ros.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "encoder_pub_demo_node");

    ros::NodeHandle nh;
    
    ros::Publisher encoderpub = nh.advertise<geometry_msgs::Point>("encoder_demo", 100);

    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;

    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / 50.0), [encoderpub, &p](const ros::TimerEvent& te){p.x += 0.01; p.y += 0.01;encoderpub.publish(p);});


    ros::spin();

    ros::shutdown();    
    return 0;
}