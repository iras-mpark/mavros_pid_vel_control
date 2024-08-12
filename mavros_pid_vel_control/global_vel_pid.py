#################################################
# NEED TO CHANGE FROM VEL_X TO VEL_LAT
#                FROM VEL_Y TO VEL_LAT
#################################################


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped

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

        self.declare_parameter('offset_lat', 0)  # 1e-5 = 약 1.1m 오프셋 (위도)
        self.declare_parameter('offset_lon', 0)  # 1e-5 = 약 1.1m 오프셋 (경도)
        self.declare_parameter('offset_alt', 4.0)  # 목표 고도 오프셋

        self.declare_parameter('vel_p', 0.1)
        self.declare_parameter('vel_i', 0.0)
        self.declare_parameter('vel_d', 0.1)

        self.offset_lat = self.get_parameter('offset_lat').value
        self.offset_lon = self.get_parameter('offset_lon').value
        self.offset_alt = self.get_parameter('offset_alt').value

        self.vel_p = self.get_parameter('vel_p').value
        self.vel_i = self.get_parameter('vel_i').value
        self.vel_d = self.get_parameter('vel_d').value

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
        self.cmd_publisher = self.create_publisher(
            TwistStamped,
            'mavros/setpoint_velocity/cmd_vel',
            qos_profile_for_pub
        )

        self.lat_controller = PIDController(self.vel_p, self.vel_i, self.vel_d)
        self.lon_controller = PIDController(self.vel_p, self.vel_i, self.vel_d)
        self.alt_controller = PIDController(1.0, 0.0, 0.1)

        self.prev_time = self.get_clock().now()
        self.current_pose = None
        self.current_pose_init = None
        self.target_pose = None
        self.target_pose_init = None

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.current_pose_init is None:
            self.current_pose_init = msg


    def target_callback(self, msg):
        self.target_pose = msg
        if self.target_pose_init is None:
            self.target_pose_init = msg

    def control_loop(self):
        if (self.current_pose is None or self.target_pose is None or
           self.current_pose_init is None or self.target_pose_init is None):
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        init_offset_lat = self.current_pose_init.latitude - self.target_pose_init.latitude
        init_offset_lon = self.current_pose_init.longitude - self.target_pose_init.longitude


        current_lat = self.current_pose.latitude
        current_lon = self.current_pose.longitude
        current_alt = self.current_pose.altitude

        target_lat = self.target_pose.latitude + self.offset_lat + init_offset_lat
        target_lon = self.target_pose.longitude + self.offset_lon + init_offset_lon
        target_alt = self.offset_alt + self.current_pose_init.altitude

        print(target_lat-current_lat)

        v_lat = self.lat_controller.compute(target_lat*1e5, current_lat*1e5, dt) # lat from [degree] to [m]
        v_lon = self.lon_controller.compute(target_lon*1e5, current_lon*1e5, dt) # lon from [degree] to [m]
        v_alt = self.alt_controller.compute(target_alt, current_alt, dt)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = v_lon
        twist_msg.twist.linear.y = v_lat
        twist_msg.twist.linear.z = v_alt

        self.cmd_publisher.publish(twist_msg)
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    hovering_controller = HoveringController()
    timer_period = 0.1  # seconds
    hovering_controller.create_timer(timer_period, hovering_controller.control_loop)
    rclpy.spin(hovering_controller)
    hovering_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
