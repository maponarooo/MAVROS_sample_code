############################################################################
# Copyright. QUAD Drone Lab.
# E-Mail. maponarooo@naver.com
# Commercial use or unauthorized copying of this code is prohibited by law.
############################################################################
#!/usr/bin/env python3
# ROS python API
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Twist
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Float32MultiArray
from collections import deque
import statistics
from std_msgs.msg import Int16MultiArray

#import tf
#from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Flight modes class
# Flight modes are activated using ROS services

class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s"%e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)



class Controller:
    # initialization method
    def __init__(self):

        self.flight_mode = 0 # 0 = Normal mode, 1 = Landing mode, 2 = return mode
        
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        self.tp = PoseStamped()
        self.tp.header.stamp = rospy.Time.now() 
        #self.tp.header.frame_id = 1
        self.tp.pose.position.x = 0
        self.tp.pose.position.y = 0
        self.tp.pose.position.z = 0.8

        self.land_x = 0
        self.land_y = 0

        self.bridge = CvBridge()
        self.cv_image_front = np.zeros((240,320,3))
        self.cv_image_lower = np.zeros((240,320,3))
        # Instantiate a cmd_vel message
        self.cmd_vel = Twist()

        self.target_error_front = Float32MultiArray()
        self.target_error_lower = Float32MultiArray()
        self.target_error_front.data = [0,0,0]
        self.target_error_lower.data = [0,0,0]

        self.target_error_front_x_deq = deque()
        self.target_error_front_y_deq = deque()
        # set the flag to use position setpoints and yaw angle
        #self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 3.0
        # update the setpoint message with the required altitude
        #self.sp.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 1.0)
        self.local_orientation_w = 0
        

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.position.z = 0.8

        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0
        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        #self.orientation_q = msg.pose.pose.orientation
        #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        #(self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
        #print(self.roll)
        #print(self.pitch)
        #print(self.yaw)

    def homeCb(self, msg):
        self.home_set = True
        self.global_home_position.latitude = msg.geo.latitude 
        self.global_home_position.longitude = msg.geo.longitude


    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.tp.pose.position.x = self.local_pos.x
        self.tp.pose.position.y = self.local_pos.y

    def x_dir(self):
    	self.tp.pose.position.x = self.local_pos.x + 0.5
    	self.tp.pose.position.y = self.local_pos.y

    def neg_x_dir(self):
    	self.tp.pose.position.x = self.local_pos.x - 0.5
    	self.tp.pose.position.y = self.local_pos.y

    def x_dir_prcision(self):
    	self.tp.pose.position.x = self.local_pos.x + 0.1
    	self.tp.pose.position.y = self.local_pos.y

    def neg_x_dir_prcision(self):
    	self.tp.pose.position.x = self.local_pos.x - 0.1
    	self.tp.pose.position.y = self.local_pos.y

    def y_dir(self): # Left
    	self.tp.pose.position.x = self.local_pos.x
    	self.tp.pose.position.y = self.local_pos.y + 0.5

    def neg_y_dir(self):  # Right
    	self.tp.pose.position.x = self.local_pos.x
    	self.tp.pose.position.y = self.local_pos.y - 0.5

    def y_dir_precision(self): # Left
    	self.tp.pose.position.x = self.local_pos.x
    	self.tp.pose.position.y = self.local_pos.y + 0.15

    def neg_y_dir_precision(self):  # Right
    	self.tp.pose.position.x = self.local_pos.x
    	self.tp.pose.position.y = self.local_pos.y - 0.15

    def z_dir(self): # Up
    	self.tp.pose.position.z = self.local_pos.z + 0.2

    def neg_z_dir(self):  # Down
    	self.tp.pose.position.z = self.local_pos.z - 0.2

    def yaw_dir(self):
    	self.tp.pose.orientation.w = self.local_orientation_w + 0.5

    def neg_yaw_dir(self):
    	self.tp.pose.orientation.w = self.local_orientation_w - 0.5

    def teleop_Cb(self, msg):
        self.cmd_vel.linear.x = msg.linear.x
        self.cmd_vel.linear.y = msg.linear.y
        self.cmd_vel.linear.z = msg.linear.z
        self.cmd_vel.angular.x = msg.angular.x
        self.cmd_vel.angular.y = msg.angular.y
        self.cmd_vel.angular.z = msg.angular.z

    def cv_cb_front(self, image_msg):
        self.cv_image_front = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imshow('Front', self.cv_image_front)
        cv2.waitKey(1)
    
    def cv_cb_lower(self, image_msg):
        self.cv_image_lower = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imshow('Lower', self.cv_image_lower)
        cv2.waitKey(1)

    def get_error_cb_front(self, msg):
        self.target_error_front_x_deq.append(msg.data[0])
        self.target_error_front_y_deq.append(msg.data[1])
        
        if len(self.target_error_front_x_deq) > 50 :
            self.target_error_front_x_deq.popleft()

        if len(self.target_error_front_y_deq) > 50 :
            self.target_error_front_y_deq.popleft()

        if abs(statistics.mean(self.target_error_front_x_deq) - msg.data[0]) < 80:
            self.target_error_front.data[0] = msg.data[0]
        
        if abs(statistics.mean(self.target_error_front_y_deq) - msg.data[1]) < 60:
            self.target_error_front.data[1] = msg.data[1]

        self.target_error_front.data[2] = msg.data[2]
        #if target_error_front.data[0] > 0:
            #print("Right")
        #else:
            #print("Left")

    def get_error_cb_lower(self, msg):
        self.target_error_lower = msg

local_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=5)
led_pub = rospy.Publisher('/led/control', Int16MultiArray, queue_size=5)
led_array = Int16MultiArray()
led_array.data = [0,0,0]

def Normal_mode(cnt, modes):
    rate = rospy.Rate(3.0)
    cnt.updateSp()
    i = 0
    while i < 20:
        print("Forward")
        cnt.updateSp()
        cnt.x_dir()
        local_pose_pub.publish(cnt.tp)
        rate.sleep()    
        i = i + 1
    i = 0
    while i <7:
        print("Left")
        cnt.updateSp()
        cnt.y_dir()
        local_pose_pub.publish(cnt.tp)
        rate.sleep()    
        i = i + 1
    i = 0
    while i < 15:
        print("Down")
        cnt.updateSp()
        cnt.neg_z_dir()
        local_pose_pub.publish(cnt.tp)
        rate.sleep()    
        i = i + 1
    i = 0
    while i < 2:
        print("Forward")
        cnt.updateSp()
        cnt.x_dir()
        local_pose_pub.publish(cnt.tp)
        rate.sleep()    
        i = i + 1
    i = 0
    while i < 5:
        print("Stay")
        cnt.updateSp()
        local_pose_pub.publish(cnt.tp)
        rate.sleep()    
        i = i + 1
    i = 0

    while i < 80:
        #print("control_faze")
        
        #print(cnt.target_error_front.data[2])
        if cnt.target_error_front.data[0] < 0 and abs(cnt.target_error_front.data[0] - 0) > 5 and cnt.target_error_front.data[2] == 1 :
                
            #print(cnt.target_error_front.data[0])
            print("control Right")    
            cnt.updateSp()
            cnt.neg_y_dir_precision()
            
            print(cnt.target_error_front.data[1])
            if cnt.target_error_front.data[1] > -30 and cnt.target_error_front.data[2] == 1 :
                #print(cnt.target_error_front.data[1])
                cnt.x_dir_prcision()
                print("control Front")

            local_pose_pub.publish(cnt.tp)
            rate.sleep()  
        elif cnt.target_error_front.data[0] > 0 and abs(cnt.target_error_front.data[0] - 0) > 5 and cnt.target_error_front.data[2] == 1:

            #print(cnt.target_error_front.data[0])
            print("control Left")
            print(cnt.target_error_front.data[1])
            cnt.updateSp()
            cnt.y_dir_precision()
            
            if cnt.target_error_front.data[1] > -30 and cnt.target_error_front.data[2] == 1 :
                #print(cnt.target_error_front.data[1])
                cnt.x_dir_prcision()
                print("control Front")

            local_pose_pub.publish(cnt.tp)
            rate.sleep()  
        else:
            cnt.updateSp()
            local_pose_pub.publish(cnt.tp)
            rate.sleep() 
        i = i + 1
    i = 0
    while i < 23:
        print("Land mode")
        cnt.updateSp()
        cnt.x_dir_prcision()
        #cnt.neg_z_dir_prcision()
        local_pose_pub.publish(cnt.tp)
        rate.sleep() 
        i = i + 1
    print("Landing!!!")    
    cnt.land_x = cnt.local_pos.x
    cnt.land_y = cnt.local_pos.y
    modes.setAutoLandMode()
    

def return_mode(cnt, modes):
    rate = rospy.Rate(3.0)
    k=0
    while k < 20:
        cnt.updateSp()
        cnt.tp.header.stamp = rospy.Time.now() 
        local_pose_pub.publish(cnt.tp)
        print("land_Local_pos")
        print(cnt.local_pos)
        #sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1
    
    # activate OFFBOARD mode
    modes.setOffboardMode()
    print("Set offboard!")

    # Make sure the drone is armed
   
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    
    print("Land take off start!!")
    
    cnt.tp.pose.position.z = 1.2
    cnt.tp.pose.position.x = 0
    cnt.tp.pose.position.y = 0
    local_pose_pub.publish(cnt.tp)
    
# Main function
def main():
    # initiate node
    rospy.init_node('control_node', anonymous=True)
    
    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(3.0)
    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    #rospy.Subscriber('/mavros/home_position/home', HomePosition, cnt.homeCb)
    rospy.Subscriber('cmd_vel', Twist, cnt.teleop_Cb)
    rospy.Subscriber("/result_img/front", Image, cnt.cv_cb_front)
    rospy.Subscriber("/result_img/lower", Image, cnt.cv_cb_lower)

    rospy.Subscriber("/target_error/front", Float32MultiArray, cnt.get_error_cb_front)
    rospy.Subscriber("/target_error/lower", Float32MultiArray, cnt.get_error_cb_lower)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    #local_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    #cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
    #thrust_pub = rospy.Publisher('/mavros/setpoint_attitude/thrust', Thrust, queue_size=1)
    
    k=0
    l = 0
    while(l < 3):
        c = 0
        led_array.data[l] = 1
        led_pub.publish(led_array)
        while c < 10:
            rate.sleep()
            c = c + 1
        led_array.data[l] = 0
        led_pub.publish(led_array)           
        l = l + 1
    
    while k < 20:
        
        cnt.tp.pose.position.x = 0
        cnt.tp.pose.position.y = 0
        cnt.tp.pose.position.z = 1.3
        cnt.tp.header.stamp = rospy.Time.now() 
        local_pose_pub.publish(cnt.tp)
        #sp_pub.publish(cnt.sp)
        rate.sleep()
        
        k = k + 1
    
    # activate OFFBOARD mode
    modes.setOffboardMode()
    print("Set offboard!")

    # Make sure the drone is armed
   
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    
    j = 0
    l = 0
    # ROS main loop
    while not rospy.is_shutdown():
        
        if cnt.local_pos.z > 1.2:
            if cnt.flight_mode == 0:
                Normal_mode(cnt, modes)
                cnt.flight_mode = 2
        
        if cnt.flight_mode == 2:
            return_mode(cnt,modes)
            cnt.flight_mode = 3
        
        """
        if cnt.local_pos.z > 0.6:
            while j < 10:
                cnt.updateSp()
                cnt.yaw_dir()
                #cnt.neg_y_dir()
                local_pose_pub.publish(cnt.tp)
                rate.sleep()
                print("Call neg_y_dir!!!") 
                j = j + 1
            while j < 20:
                cnt.updateSp()
                cnt.yaw_dir()
                #cnt.neg_x_dir()
                local_pose_pub.publish(cnt.tp)
                rate.sleep()
                print("Call neg_x_dir!!!")
                j = j + 1
            while j < 30:
                cnt.updateSp()
                cnt.neg_yaw_dir()
                #cnt.y_dir()
                local_pose_pub.publish(cnt.tp)
                rate.sleep()
                print("Call y_dir!!!")
                j = j + 1
            while j < 40:
                cnt.updateSp()
                cnt.neg_yaw_dir()
                #cnt.x_dir()
                local_pose_pub.publish(cnt.tp)
                rate.sleep()
                print("Call x_dir!!!")
                j = j + 1
        """
        #cmd_vel_pub.publish(cnt.cmd_vel)
        """
        if cnt.local_pos.z > 0.7:
            while l < 10:
                cnt.updateSp()
                print("Up")
                #cnt.z_dir()
                cnt.neg_y_dir()
                local_pose_pub.publish(cnt.tp)
                rate.sleep()
                l = l + 1
        """
        if cnt.local_pos.z > 2.3:
            print("Z over!!! AutoLand")
            modes.setAutoLandMode()
        
        cnt.tp.header.stamp = rospy.Time.now() 
        print("Target Pose")
        print(cnt.tp.pose.position)
        local_pose_pub.publish(cnt.tp)
        print("Local pose")
        rospy.loginfo("local_pos_x : %s", cnt.local_pos.x)
        rospy.loginfo("local_pos_y : %s", cnt.local_pos.y)
        rospy.loginfo("local_pos_z : %s", cnt.local_pos.z)
        #rospy.loginfo("local_orientation_w : %s", cnt.local_orientation_w)
 

        rate.sleep()


if __name__ == '__main__':
    try:
        main() 
    except rospy.ROSInterruptException:
        print("shutdown")
        pass
