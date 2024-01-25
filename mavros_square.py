import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Twist
import time

def set_velocity(velocity_publisher, x=0, y=0, z=0, yaw=0):
    vel_msg = Twist()
    vel_msg.linear.x = x
    vel_msg.linear.y = y
    vel_msg.linear.z = z
    vel_msg.angular.z = yaw
    velocity_publisher.publish(vel_msg)

def takeoff_sequence(local_pos_pub, current_pose, height):
    rospy.loginfo("Taking off...")
    pose = PoseStamped()
    pose.pose.position.x = current_pose.pose.position.x
    pose.pose.position.y = current_pose.pose.position.y
    pose.pose.position.z = height
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    rospy.loginfo("Takeoff finished")

def land_sequence(land_client):
    rospy.loginfo("Landing...")
    resp1 = land_client.call()
    if resp1.success:
        rospy.loginfo("Landing sequence finished")

def fly_square(velocity_publisher):
    rospy.loginfo("Flying square path...")
    set_velocity(velocity_publisher, x=1)
    time.sleep(3)
    set_velocity(velocity_publisher, y=1)
    time.sleep(3)
    set_velocity(velocity_publisher, x=-1)
    time.sleep(3)
    set_velocity(velocity_publisher, y=-1)
    time.sleep(3)
    rospy.loginfo("Square path finished")

rospy.init_node('fly_square', anonymous=True)
rate = rospy.Rate(20) # 20hz

# Connect to services for arming and changing flight mode
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandBool)

# Connect to publisher for setting velocity
velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

# Connect to subscriber for getting current state and pose
current_state = State()
current_pose = PoseStamped()
def state_callback(state):
    global current_state
    current_state = state
state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
def pose_callback(pose):
    global current_pose
    current_pose = pose
pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)

# Wait for FCU connection
while not rospy.is_shutdown() and not current_state.connected:
    rate.sleep()

# Set offboard mode and arm
set_mode_client(base_mode=0, custom_mode="OFFBOARD")
arming_client(True)

# Takeoff
takeoff_sequence(local_pos_pub, current_pose, 3)

# Fly square path
fly_square(velocity_publisher)

# Land
land_sequence(land_client)
