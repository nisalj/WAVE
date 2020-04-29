#!/usr/bin/env python

import rospy
import numpy as np
import math
from vesc_msgs.msg import VescStateStamped
from vesc_msgs.msg import VescState
from std_msgs.msg import String
from std_msgs.msg import Float64

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist


rospy.init_node('diff_drive', anonymous=True)
gyro_enabled = rospy.get_param('~gyro_enabled')
PID_enabled = rospy.get_param('~PID_enabled')
left_wheel_topic_control = rospy.get_param('~left_wheel_topic_control', '/left_wheel/ref')
right_wheel_topic_control = rospy.get_param('~right_wheel_topic_control', '/right_wheel/ref')
right_wheel_topic_sense = rospy.get_param('~right_wheel_topic_sense', '/right_wheel/sense')
left_wheel_topic_sense = rospy.get_param('~left_wheel_topic_sense', '/left_wheel/sense')

r = 0.127 #5 inches
b = 0.260 #255 mm
motor_poles = 15
A = [[1/r, 0, b/r], [1/r,0,-b/r]]
B = [[r/2, r/2], [0,0], [r/(2*b), -r/(2*b)]]
rad_per_sec_to_erpm_conversion_factor = (60/(2*math.pi))*motor_poles;
right_wheel_sensed_eprm = 0
right_wheel_sensed_rad = 0
left_wheel_sensed_eprm = 0
left_wheel_sensed_rad = 0
w_sensed = 0
w_ref = 0
w_control = 0
x_dot_sensed  = 0 
x_dot_control = 0 
x_dot_ref = 0
right_wheel_sense_pub_m = rospy.Publisher('/sensors/right_wheel', Float64, queue_size=10)
left_wheel_sense_pub_m = rospy.Publisher('/sensors/left_wheel', Float64, queue_size=10)
right_wheel_sense_pub = rospy.Publisher(right_wheel_topic_sense, Float64, queue_size = 10)
left_wheel_sense_pub = rospy.Publisher(left_wheel_topic_sense, Float64, queue_size = 10)
right_wheel_control_pub = rospy.Publisher(right_wheel_topic_control, Float64, queue_size=1)
left_wheel_control_pub = rospy.Publisher(left_wheel_topic_control, Float64, queue_size=1)
base_sense_pub = rospy.Publisher('/mobile_base_controller/sensed_vel', Twist, queue_size = 10) 

vel_msg = Twist()





#PID topics
#control_topic -> mobile_base_controller/cmd_vel
#ref_topic -> mobile_base_controller/ref_vel
#sensor_topic -> mobile_base_controller/sensed_vel

#subscribe to right_wheel_sensors/core/ -> updated wheel speed variable
#subscribe to left_wheel_sensors/core/ -> update wheel speed variable
#subscribe to imu -> update w, calculate base vel

#get reference geometery twist msg and publish to ref topic for each wheel, from teleop 
def ref_vel_callback(data):
    wheels = base_to_wheel_speeds(data.linear.x, 0, data.angular.z)
    wheel1Speed = -rad_per_sec_to_erpm_conversion_factor * wheels[0]
    wheel2Speed =  rad_per_sec_to_erpm_conversion_factor * wheels[1]
#    if(abs(wheel1Speed) > 500):
#	if(wheel1Speed > 0):
#		wheel1Speed = 500
#	else:
#		wheel1Speed = -500
#    if(abs(wheel2Speed) > 500):
#	if(wheel2Speed > 0):
#		wheel2Speed = 500
#	else:
#		wheel2Speed = -500	
#	
    rospy.loginfo("right %f, left %f" , wheel1Speed, wheel2Speed)
    right_wheel_control_pub.publish(wheel1Speed)
    left_wheel_control_pub.publish(wheel2Speed)
#    publish_sensed_vel()

def publish_sensed_vel():
    wheels_to_base_velocity(-right_wheel_sensed_rad, left_wheel_sensed_rad)
    if(gyro_enabled is True):
        publish_sensed_vel_wheels()
    publish_sensed_vel_base()


#publish the sensed vel for each wheel
def publish_sensed_vel_wheels():
    wheels = base_to_wheel_speeds(x_dot_sensed, 0, w_sensed)
    right_wheel_sensed_erpm_m = rad_per_sec_to_erpm_conversion_factor * wheels[0]  
    left_wheel_sensed_erpm_m = rad_per_sec_to_erpm_conversion_factor * wheels[1]
    right_wheel_sense_pub.publish(-right_wheel_sensed_erpm_m)
    left_wheel_sense_pub.publish(left_wheel_sensed_erpm_m)

def publish_sensed_vel_base():
    global vel_msg
    vel_msg.linear.x = x_dot_sensed
    vel_msg.angular.z = w_sensed
    base_sense_pub.publish(vel_msg)
 
def right_wheel_callback(data):
   # speed = data.VescState.speed
    global right_wheel_sensed_eprm
    global right_wheel_sensed_rad
    right_wheel_sensed_eprm = data.state.speed
    right_wheel_sense_pub.publish(right_wheel_sensed_eprm)
    right_wheel_sensed_rad = right_wheel_sensed_eprm / rad_per_sec_to_erpm_conversion_factor
    publish_sensed_vel()
    #rospy.loginfo("Right: %f",right_wheel_sensed_rad)
    #rospy.loginfo("Base: %f", wheels_to_base_velocity(right_wheel_sensed_rad,left_wheel_sensed_rad)[2])

def left_wheel_callback(data):
    global left_wheel_sensed_eprm
    global left_wheel_sensed_rad
    left_wheel_sensed_eprm = data.state.speed
    left_wheel_sense_pub.publish(left_wheel_sensed_eprm)
    left_wheel_sensed_rad = left_wheel_sensed_eprm / rad_per_sec_to_erpm_conversion_factor
    #rospy.loginfo("Left: %f",left_wheel_sensed_rad)

def imu_callback(data):
    global w_sensed 
    global x_dot_sensed
    w_sensed = data.angular_velocity.z


def base_to_wheel_speeds(x_dot, y_dot, w):
    wheels = np.dot(A, [x_dot,y_dot,w])
    return wheels
    
def wheels_to_base_velocity(right_wheel_rad, left_wheel_rad):
    global w_sensed
    global x_dot_sensed
    base = np.dot(B,[right_wheel_rad, left_wheel_rad])

    x_dot_sensed = base[0]
    if (gyro_enabled is True):
        return base
    else:
        w_sensed = base[2]
        return
                                                                  
def init():
    rospy.Subscriber("right_wheel/sensors/core", VescStateStamped, right_wheel_callback)
    rospy.Subscriber("left_wheel/sensors/core", VescStateStamped, left_wheel_callback)
    rospy.Subscriber("mobile_base_controller/cmd_vel", Twist, ref_vel_callback)
    if(gyro_enabled is True):
        rospy.Subscriber("imu", Imu, imu_callback)
    #rospy.Subscriber("mobile_base_controller/ref_vel", 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
