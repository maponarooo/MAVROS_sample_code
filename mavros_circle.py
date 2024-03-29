#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest, CommandBoolRequest
from geometry_msgs.msg import PoseStamped, Twist
from math import pow, sqrt
import math

current_state = State()
pose = PoseStamped()
radius = 3

def state_cb(msg):
    global current_state
    current_state = msg

def position_cb(msg):
    global pose
    pose = msg.pose

def calculate_distance(x1, y1, x2, y2):
    return sqrt(pow((x2-x1),2) + pow((y2-y1),2))

if __name__ == "__main__":
    rospy.init_node('offboard_node')
    rate = rospy.Rate(20.0)

    state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_pos_sub = rospy.Subscriber('mavros/local_position/local', PoseStamped, position_cb)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    while (not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 2.0

    for i in range(100):
        if(rospy.is_shutdown()):
            break
            
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    if (set_mode_client.call(offb_set_mode).mode_sent == True):
        rospy.loginfo("Offboard enabled")

    if (arming_client.call(arm_cmd).success == True):
        rospy.loginfo("Vehicle armed")

    rospy.loginfo("Taking off...")

    while not rospy.is_shutdown():
        distance = calculate_distance(pose.pose.position.x, pose.pose.position.y, 0, 0)
        if distance > radius:
            pose.pose.position.x = pose.pose.position.x/radius
            pose.pose.position.y = pose.pose.position.y/radius
        else:
            theta = math.atan2(pose.pose.position.y, pose.pose.position.x) + 0.1
            pose.pose.position.x = radius * math.cos(theta)
            pose.pose.position.y = radius * math.sin(theta)

        local_pos_pub.publish(pose)
        rate.sleep()
