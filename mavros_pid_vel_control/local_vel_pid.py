import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, pv, dt):
        error = setpoint - pv
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class HoveringController(Node):
    def __init__(self):
        super().__init__('hovering_controller')


        self.declare_parameter('offset_xy', 0.0)
        self.declare_parameter('offset_z', 4.0)

        self.offset_xy = self.get_parameter('offset_xy').value
        self.offset_z  = self.get_parameter('offset_z').value

        self.declare_parameter("vel_p_xy", 0.5)
        self.vel_p_xy = self.get_parameter('vel_p_xy').value

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        qos_profile_for_pub = QoSProfile(depth=10)
        qos_profile_for_pub.reliability = ReliabilityPolicy.RELIABLE

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'scout/odom',
            self.odom_callback,
            qos_profile
        )
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        self.cmd_publisher = self.create_publisher(
            TwistStamped,
            'mavros/setpoint_velocity/cmd_vel',
            qos_profile_for_pub
        )

        self.x_controller = PIDController(self.vel_p_xy, 0.0, 0.1)
        self.y_controller = PIDController(self.vel_p_xy, 0.0, 0.1)
        self.z_controller = PIDController(1.0, 0.0, 0.1)

        self.prev_time = self.get_clock().now()
        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def odom_callback(self, msg):
        if self.current_pose is None:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_z = self.current_pose.position.z

        target_x = msg.pose.pose.position.x - self.offset_xy
        target_y = msg.pose.pose.position.y - self.offset_xy
        target_z = msg.pose.pose.position.z + self.offset_z

        vx = self.x_controller.compute(target_x, current_x, dt)
        vy = self.y_controller.compute(target_y, current_y, dt)
        vz = self.z_controller.compute(target_z, current_z, dt)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz

        self.cmd_publisher.publish(twist_msg)
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    hovering_controller = HoveringController()
    rclpy.spin(hovering_controller)
    hovering_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

