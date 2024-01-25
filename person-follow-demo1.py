import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

bridge = CvBridge()
HOGCV = cv2.HOGDescriptor()
HOGCV.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def detect_person(frame):
    boxes, weights = HOGCV.detectMultiScale(frame, winStride=(4, 4), padding=(8, 8), scale=1.05)
    center = None

    for (x, y, w, h) in boxes:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        center = (x + w/2, y + h/2)

    return frame, center

def image_callback(msg):
    global bridge
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    frame, center = detect_person(cv_image)

    if center:
        cv2.circle(frame, center, 5, (255, 0, 0), -1)

    cv2.imshow("preview", frame)
    cv2.waitKey(1)

    return center

def controller():
    rospy.init_node('person_follower', anonymous=True)
    velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    vel_msg = Twist()

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        center = image_callback()

        if center is not None:
            err_x = center[0] - 320 # image width/2
            err_y = center[1] - 240 # image height/2

            vel_msg.linear.x = 0.5
            vel_msg.angular.z = -float(err_x) / 100

        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0

        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
