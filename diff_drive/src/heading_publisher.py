#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion

heading_angle_sensed = 0; 
heading_angle_sensed_pub = rospy.Publisher('/sensed/yaw', Float64, queue_size=10);


def publish_sensed_angle():
    global heading_angle_sensed
    heading_angle_sensed_pub.publish(heading_angle_sensed)

def imu_callback(data):
    global heading_angle_sensed
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    heading_angle_sensed =  yaw * 180/math.pi
    publish_sensed_angle()


def init():
    rospy.init_node('heading_publisher', anonymous=True)
    rospy.Subscriber("imu", Imu, imu_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
