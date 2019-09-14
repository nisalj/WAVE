#!/usr/bin/env python


import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
vel_msg = Twist()

heading_angle_sensed = 0
heading_angle_ref = -76
heading_angle_sensed_pub = rospy.Publisher('/sensed/yaw', Float64, queue_size=10)
heading_angle_ref_pub = rospy.Publisher('/ref/yaw', Float64, queue_size=10)
cmd_pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size = 10)

def publish_sensed_angle():
    global heading_angle_sensed
    heading_angle_sensed_pub.publish(heading_angle_sensed)

def publish_ref():
    global heading_angle_ref
    heading_angle_ref_pub.publish(heading_angle_ref)

def control_callback(data):
    vel_msg.linear.x = 0.5
    vel_msg.angular.z = data.data
    cmd_pub.publish(vel_msg)
    rospy.loginfo(data)
    


def imu_callback(data):
    global heading_angle_sensed
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    heading_angle_sensed =  yaw * 180/math.pi
    #if (heading_angle_sensed < 0):
    #    heading_angle_sensed = -heading_angle_sensed
    #else:
    #    heading_angle_sensed = 360-heading_angle_sensed
    #euler =  euler_from_quaternion(data.orientation)
   # heading_angle_sensed = euler[0]
    publish_sensed_angle()

def init():
    rospy.init_node('heading_controller', anonymous=True)
    rospy.Subscriber("imu", Imu, imu_callback)
    rospy.Subscriber("control/yaw", Float64, control_callback)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
       # publish_ref()
        rate.sleep()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
    
