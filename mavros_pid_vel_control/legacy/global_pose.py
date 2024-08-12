import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped


class HoveringController(Node):
    def __init__(self):
        super().__init__('hovering_controller')

        self.declare_parameter('offset_lat', 0)  # 1e-5 = 약 1.1m 오프셋 (위도)
        self.declare_parameter('offset_lon', 0)  # 1e-5 = 약 1.1m 오프셋 (경도)
        self.declare_parameter('offset_alt', 4.0)  # 목표 고도 오프셋


        self.offset_lat = self.get_parameter('offset_lat').value
        self.offset_lon = self.get_parameter('offset_lon').value
        self.offset_alt = self.get_parameter('offset_alt').value


        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        qos_profile_for_pub = QoSProfile(depth=10)
        qos_profile_for_pub.reliability = ReliabilityPolicy.RELIABLE

        self.target_subscriber = self.create_subscription(
            NavSatFix,
            'scout/fix',
            self.target_callback,
            qos_profile
        )
        self.pose_subscriber = self.create_subscription(
            NavSatFix,
            'mavros/global_position/global',
            self.pose_callback,
            qos_profile
        )
        self.cmd_pose_publisher = self.create_publisher(
            GeoPoseStamped,
            'mavros/setpoint_position/global',
            qos_profile_for_pub
        )

        self.current_pose_init = None
        self.target_pose = None
        self.target_pose_init = None

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)

    def pose_callback(self, msg):
        if self.current_pose_init is None:
            self.current_pose_init = msg

    def target_callback(self, msg):
        self.target_pose = msg
        if self.target_pose_init is None:
            self.target_pose_init = msg

    def control_loop(self):
        if (self.target_pose is None or self.current_pose_init is None or
            self.target_pose_init is None):
            return


        init_offset_lat = self.current_pose_init.latitude - self.target_pose_init.latitude
        init_offset_lon = self.current_pose_init.longitude - self.target_pose_init.longitude


        target_lat = self.target_pose.latitude + self.offset_lat + init_offset_lat
        target_lon = self.target_pose.longitude + self.offset_lon + init_offset_lon
        target_alt = self.offset_alt #+ self.current_pose_init.altitude


        geopose_msg = GeoPoseStamped()
        geopose_msg.header.stamp = self.get_clock().now().to_msg()
        geopose_msg.pose.position.latitude = target_lat
        geopose_msg.pose.position.longitude = target_lon
        geopose_msg.pose.position.altitude = target_alt

        self.cmd_pose_publisher.publish(geopose_msg)


def main(args=None):
    rclpy.init(args=args)
    hovering_controller = HoveringController()

    rclpy.spin(hovering_controller)
    hovering_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
