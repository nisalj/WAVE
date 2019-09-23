#!/usr/bin/env python


import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
vel_msg = Twist()

cmd_pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size = 10)
control_sub = None;

def toggle_pid_callback(data):
    global control_sub;
    if (data.data == True):
        control_sub = rospy.Subscriber("control/yaw", Float64, control_callback)
    else:
	control_sub.unregister();
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        cmd_pub.publish(vel_msg)

def control_callback(data):
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = data.data
    cmd_pub.publish(vel_msg)
    rospy.loginfo(data)

def init():
    rospy.init_node('heading_controller', anonymous=True)
    rospy.Subscriber("toggle_heading_pid" , Bool, toggle_pid_callback); 
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
    
