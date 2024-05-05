#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest
from geometry_msgs.msg import PoseStamped, Twist
from math import pow, sqrt
import math
import numpy as np

if __name__ == "__main__":
    rospy.init_node('mavros_square', anonymous=True)

    current_state = State()
    pose = PoseStamped()

    def state_cb(msg):
        global current_state
        current_state = msg

    def set_velocity(velocity_publisher, x=0, y=0, z=0, yaw=0):
        vel_cmd = Twist()
        vel_cmd.linear.x = x
        vel_cmd.linear.y = y
        vel_cmd.linear.z = z
        vel_cmd.angular.z = yaw
        velocity_publisher.publish(vel_cmd)

    state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(10)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2
    #pose.pose.orientation.z = 0

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    square_xy = 2.0 # the length of a side of a square.
    delta_xy = 0.0 # fly speed 
    gamma_xy = 2.0 # the length of a side of a square.
    vel_x = 0
    vel_y = 0
    vel_z = 0.5
    vel_yaw = 0

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled!")
                last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed!")
                    last_req = rospy.Time.now()
        
        if(current_state.armed and (rospy.Time.now() - last_req) >= rospy.Duration(10.0) and (rospy.Time.now() - last_req) < rospy.Duration(20.0)):
            rospy.loginfo("Vehicle Fly Square path-1.")
            vel_x = 1
            vel_y = 0
            vel_z = 0
            vel_yaw = 0
        elif(current_state.armed and (rospy.Time.now() - last_req) >= rospy.Duration(20.0) and (rospy.Time.now() - last_req) < rospy.Duration(30.0)):
            rospy.loginfo("Vehicle Fly Square path-2.")
            vel_x = 0
            vel_y = 1
            vel_z = 0
            vel_yaw = 0
        elif(current_state.armed and (rospy.Time.now() - last_req) >= rospy.Duration(30.0) and (rospy.Time.now() - last_req) < rospy.Duration(40.0)):
            rospy.loginfo("Vehicle Fly Square path-3.")
            vel_x = -1
            vel_y = 0
            vel_z = 0
            vel_yaw = 0
        elif(current_state.armed and (rospy.Time.now() - last_req) >= rospy.Duration(40.0) and (rospy.Time.now() - last_req) < rospy.Duration(50.0)):
            rospy.loginfo("Vehicle Fly Square path-4.")
            vel_x = 0
            vel_y = -1
            vel_z = 0
            vel_yaw = 0
        elif(current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(50.0)):
            rospy.loginfo("Vehicle Stoped!")
            vel_x = 0
            vel_y = 0
            vel_z = 0
            vel_yaw = 0

        set_velocity(velocity_publisher, vel_x, vel_y, vel_z, vel_yaw)
        rate.sleep()
