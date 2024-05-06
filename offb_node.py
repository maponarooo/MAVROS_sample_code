import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node')

        self.current_state = None
        self.get_logger().info('Offboard node has been started.')

        self.state_sub = self.create_subscription(
            State, 
            'mavros/state', 
            self.state_cb, 
            10)

        self.local_pos_pub = self.create_publisher(
            PoseStamped, 
            'mavros/setpoint_position/local', 
            10)

        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.2

        self.offb_set_mode = SetMode.Request()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBool.Request()
        self.arm_cmd.value = True

        self.last_req = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def state_cb(self, msg):
        self.current_state = msg

    def timer_callback(self):
        now = self.get_clock().now()

        if (self.current_state is None or not self.current_state.connected):
            return

        if(self.current_state.mode != "OFFBOARD" and (now - self.last_req) > Duration(seconds=5.0)):
            resp = self.set_mode_client.call(self.offb_set_mode)
            if resp and resp.mode_sent:
                self.get_logger().info('OFFBOARD enabled')
            self.last_req = now
        else:
            if(not self.current_state.armed and (now - self.last_req) > Duration(seconds=5.0)):
                resp = self.arming_client.call(self.arm_cmd)
                if resp and resp.success:
                    self.get_logger().info('Vehicle armed')
                self.last_req = now

        self.local_pos_pub.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    offb_node = OffboardNode()
    rclpy.spin(offb_node)
    offb_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
  
