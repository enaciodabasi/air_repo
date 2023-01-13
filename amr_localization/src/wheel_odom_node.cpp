#include "../include/amr_localization/wheel_odom_node.hpp"

WheelOdomNode::WheelOdomNode()
    : NodeHandle()
{

    m_WheelEncoderSubscriber = this->subscribe(
        "wheel_encoders_rl",
        uint32_t(100),
        &WheelOdomNode::calculateMovement,
        this,
        ros::TransportHints().tcpNoDelay()
    );

    m_OdomPublisher = this->advertise<nav_msgs::Odometry>("wheel_odom", 100);
}

double WheelOdomNode::normalize_angle(double angle)
{
    if(angle > M_PI)
    {
        return (angle -= 2*M_PI);
    }
    else if(angle < -M_PI)
    {
        return (angle += 2*M_PI);
    }
    
    return angle;
}

void WheelOdomNode::update_odom()
{
    double cycleDistance = (m_WheelDistanceRight + m_WheelDistanceLeft) / 2.0;
    double cycleAngle = std::asin((m_WheelDistanceRight - m_WheelDistanceLeft) / m_WheelBase);
    double averageAngle = (cycleAngle / 2.0) + m_OldTheta;

    double averageAngle_Normalized = normalize_angle(averageAngle);

    m_NewOdom.pose.pose.position.x = m_OldX + std::cos(averageAngle_Normalized) * cycleDistance;
    m_NewOdom.pose.pose.position.y = m_OldY + std::sin(averageAngle_Normalized) * cycleDistance;
    m_NewOdom.pose.pose.position.z = 0;

    m_OldTheta += cycleAngle;
    m_OldTheta = normalize_angle(m_OldTheta);

    tf2::Quaternion quat;
    quat.setRPY(0, 0, m_OldTheta);
    m_NewOdom.pose.pose.orientation.x = quat.x();
    m_NewOdom.pose.pose.orientation.y = quat.y();
    m_NewOdom.pose.pose.orientation.z = quat.z();
    m_NewOdom.pose.pose.orientation.w = quat.w();

    m_LastTime = ros::Time::now();

    double dt = (m_CurrentTime - m_LastTime).toSec();

    m_NewOdom.header.stamp = m_LastTime;
    m_NewOdom.twist.twist.linear.x = cycleDistance/dt;
    m_NewOdom.twist.twist.linear.y = 0;
    m_NewOdom.twist.twist.linear.z = 0;
    m_NewOdom.twist.twist.angular.x = 0;
    m_NewOdom.twist.twist.angular.y = 0;
    m_NewOdom.twist.twist.angular.z = cycleAngle/dt;

    m_NewOdom.header.frame_id = "odom";
    m_OdomPublisher.publish(m_NewOdom);

    m_OldX = m_NewOdom.pose.pose.position.x;
    m_OldY = m_NewOdom.pose.pose.position.y;
    m_OldOdom = m_NewOdom;

}

void WheelOdomNode::calculateMovement(const geometry_msgs::Point::ConstPtr& encoders_rl)
{
    if(m_InitialEncoderReceived)
    {
        m_CurrentTime = ros::Time::now();

        m_WheelDistanceRight = (encoders_rl->x - m_LastRightEncoder) / 1000.0;
        m_WheelDistanceLeft = (encoders_rl->y - m_LastLeftEncoder) / 1000.0;
        this->update_odom();
    }
    else
    {
        m_InitialEncoderReceived = true;
        m_CurrentTime = ros::Time::now();
        m_LastTime = m_CurrentTime;
    }

    m_LastRightEncoder = encoders_rl->x;
    m_LastLeftEncoder = encoders_rl->y;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_odom_node");

    WheelOdomNode wheelOdomNode;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}