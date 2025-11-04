# follower.py (ROS 2 Jazzy / rclpy)
import math
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarFollower(Node):
    def __init__(self):
        super().__init__('lidar_follower')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/world/my_world2/model/vehicle_blue/link/chassis/sensor/gpu_lidar/scan',
            self.scan_cb, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)

        # Params
        self.declare_parameter('d_des', 3.0)      # desired gap (m)
        self.declare_parameter('k_r', 0.5)        # distance gain
        self.declare_parameter('k_phi', 1.2)      # angle gain
        self.declare_parameter('fov_deg', 30.0)   # front sector total FOV

        self.timer = self.create_timer(0.05, lambda: None)  # keep node alive
        self.last_cmd = Twist()

    def scan_cb(self, msg: LaserScan):
        d_des  = self.get_parameter('d_des').value
        k_r    = self.get_parameter('k_r').value
        k_phi  = self.get_parameter('k_phi').value
        fov    = math.radians(self.get_parameter('fov_deg').value)

        # Front sector indices
        angles = [msg.angle_min + i*msg.angle_increment for i in range(len(msg.ranges))]
        front = [(i, a, msg.ranges[i]) for i, a in enumerate(angles) if -fov/2 <= a <= fov/2]

        # Filter finite ranges
        front = [(i, a, r) for (i,a,r) in front if math.isfinite(r)]
        cmd = Twist()

        if not front:
            # simple search behavior
            cmd.angular.z = 0.3
            cmd.linear.x  = 0.1
        else:
            # take the minimum range beam in front sector
            i_min, phi, r_min = min(front, key=lambda t: t[2])

            # Distance control
            v = k_r * (r_min - d_des)
            # Heading control (steer to bearing)
            w = k_phi * phi

            # Safety shaping
            if r_min < 1.5:
                v = min(v, 0.1)
            if abs(phi) > 0.5:
                v = min(v, 0.15)

            # Saturations (match your diff drive limits)
            v = max(0.0, min(0.45, v))        # no reverse for simplicity
            w = max(-0.8, min(0.8, w))

            cmd.linear.x  = v
            cmd.angular.z = w

        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

def main():
    rclpy.init()
    node = LidarFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
