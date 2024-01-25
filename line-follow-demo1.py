import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool, SetMode

bridge = CvBridge()
line_dir = 0

def image_callback(msg):
    global bridge, line_dir
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        rows,cols = cv_image.shape[:2]
        [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
        lefty = int((-x*vy/vx) + y)
        righty = int(((cols-x)*vy/vx)+y)
        cv2.line(cv_image,(cols-1,righty),(0,lefty),(0,255,0),2)

        line_dir = - vy / vx

    cv2.imshow("shapes", cv_image)
    cv2.waitKey(1)

def controller():
    rospy.init_node('line_follower', anonymous=True)
    velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    vel_msg = Twist()

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        vel_msg.linear.x = 0.5
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = line_dir * 0.3

        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        image_sub = rospy.Subscriber("/camera/image_raw", Image, image_callback)
        controller()
    except rospy.ROSInterruptException:
        pass
