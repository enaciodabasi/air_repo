#!/usr/bin/env python

import pyads
import rospy
import threading
from threading import Lock
from geometry_msgs.msg import Twist, Point

lock = Lock()

def add_route(
    local_address='192.168.66.200.1.1',
    AMS='192.168.66.200.1.1', 
    PLC='192.168.66.10',
    USER='Administrator', 
    PW='1', 
    HOSTNAME='192.168.66.200', 
    PLC_AMS_ID='192.168.6.10.1.1'):

    pyads.open_port()
    pyads.set_local_address(local_address)

    pyads.add_route_to_plc(AMS, HOSTNAME, PLC, USER, PW)


def cmd_vel_listener(vel_msg):

    rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", str(vel_msg.linear.x), str(vel_msg.angular.z))
    lock.acquire()
    plc.write_by_name('GVL.fRefLin', vel_msg.linear.x, pyads.PLCTYPE_LREAL)
    plc.write_by_name('GVL.fRefAng', vel_msg.angular.z, pyads.PLCTYPE_LREAL)
    lock.release()

def encoder_listener():
    encoder_pub = rospy.Publisher('wheel_encoders_rl', Point, queue_size=10)
    rate = rospy.Rate(50) # 50hz

    while not rospy.is_shutdown():
        wheel_encoders_rl_msg = Point()
        lock.acquire()
        wheel_encoders_rl_msg.x = plc.read_by_name("GVL.fActPosR", pyads.PLCTYPE_LREAL)
        wheel_encoders_rl_msg.y = plc.read_by_name("GVL.fActPosL", pyads.PLCTYPE_LREAL)
        lock.release()
        rospy.loginfo(rospy.get_caller_id() + "I read %s %s", str(wheel_encoders_rl_msg.x), str(wheel_encoders_rl_msg.y))
        encoder_pub.publish(wheel_encoders_rl_msg)
        rate.sleep()

def __del__(self): 
    encoder_thread.join()
    # Close Connection
    plc.close()
    print ("Connection Closed")

if __name__ == '__main__':
    plc = pyads.Connection('192.168.6.10.1.1', pyads.PORT_TC3PLC1, ip_address="192.168.6.10")  # BeckHoff AmsNetId
    # Connect to BeckHoff
    plc.open()
    if (plc.is_open):
        print ("PLC CONNECTED!")
	    # Init ROS Node
        rospy.init_node('amr_ros_ads')
        rospy.Subscriber("cmd_vel", Twist, cmd_vel_listener, queue_size = 1)
        encoder_thread = threading.Thread(target=encoder_listener)
        encoder_thread.start()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    #add_route()