#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np

class ParkingLogicNode(Node):
    def __init__(self):
        super().__init__('parking_logic_node')
        self.odom_sub = self.create_subscription(Odometry, '/model/turtlebot3/odometry', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/model/turtlebot3/scan/points', self.lidar_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/model/turtlebot3/cmd_vel', 10)

        self.current_pose = None
        self.lidar_data = None
        self.timer = self.create_timer(0.1, self.park_logic)

        # Parkolóhely keresési paraméterek
        self.target_found = False
        self.target_distance = 1.5  # ennyi méterre parkoljon le
        self.obstacle_threshold = 0.5  # ha közelebb van akadály, megáll

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def lidar_callback(self, msg):
        points = []
        self.get_logger().info(f'Pontok száma szűrés előtt: {len(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))}')
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if (-0.4 < x < 0.4) and (-0.4 < y < 0.4):
                continue
            if z < -0.2:
                continue
            points.append((x, y, z))
        points = np.array(points)
        self.get_logger().info(f'LIDAR pontok száma: {len(points)}')

        # Szűrt tömbök
        front_mask = (points[:,0] > 0) & (np.abs(points[:,1]) < 0.3)
        left_mask = (points[:,1] > 0.5) & (np.abs(points[:,0]) < 0.5)
        right_mask = (points[:,1] < -0.5) & (np.abs(points[:,0]) < 0.5) 
        right_front_mask = (points[:,0] > 0.5) & (points[:,1] < -0.5)
        right_side_mask = (points[:,1] < -0.5) & (np.abs(points[:,0]) < 0.5)

        min_front = np.min(points[front_mask, 0]) if np.any(front_mask) else None
        min_left = np.min(points[left_mask, 1]) if np.any(left_mask) else None
        min_right = np.min(points[right_mask, 1]) if np.any(right_mask) else None
        min_right_front = np.min(points[right_front_mask, 0]) if np.any(right_front_mask) else None
        min_right_side = np.min(points[right_side_mask, 1]) if np.any(right_side_mask) else None

        self.lidar_data = (min_front, min_left, min_right, min_right_front, min_right_side)

        # U-alakú parkoló detektálásához szükséges értékek eltárolása
        right_points = points[(points[:,1] < -0.6) & (np.abs(points[:,0]) < 0.6)]
        entrance_points = points[(points[:,0] > 0.6) & (np.abs(points[:,1]) < 0.6)]
        back_points = points[(points[:,0] < -0.4) & (np.abs(points[:,1]) < 0.6)]
        self.right_points_len = len(right_points)
        self.entrance_points_len = len(entrance_points)
        self.back_points_len = len(back_points)

        # Logolás a detektáláshoz
        self.get_logger().info(
            f'right_points_len={self.right_points_len}, entrance_points_len={self.entrance_points_len}, back_points_len={self.back_points_len}'
        )
        self.get_logger().info(f'right_points példák: {right_points[:5]}')
        self.get_logger().info(f'entrance_points példák: {entrance_points[:5]}')
        self.get_logger().info(f'back_points példák: {back_points[:5]}')

        # Ha két oldalfal van, de elöl nincs pont (bejárat), akkor U-alakú parkoló
        if len(right_points) > 10 and len(entrance_points) < 3:
            self.get_logger().info('U alakú parkolóhelyet találtam jobbra!')
            

    def park_logic(self):
        self.get_logger().info('park_logic timer meghívva')
        self.get_logger().info(
            f'park_logic fut | target_found={self.target_found} | right_points_len={getattr(self, "right_points_len", None)} | entrance_points_len={getattr(self, "entrance_points_len", None)}'
        )
        if self.current_pose is None or self.lidar_data is None:
            self.get_logger().info('Nincs aktuális pozíció vagy LIDAR adat!')
            return

        min_front, min_left, min_right, min_right_front, min_right_side = self.lidar_data
        twist = Twist()

        if not self.target_found:
            if hasattr(self, "right_points_len") and hasattr(self, "entrance_points_len"):
                self.get_logger().info(
                    f'U-hely keresés: right_points_len={self.right_points_len}, entrance_points_len={self.entrance_points_len}'
                )
                if self.right_points_len > 100 and self.entrance_points_len < 350:
                    self.get_logger().info('U alakú parkolóhelyet találtam, odafordulok!')
                    twist.angular.z = -0.5
                    self.target_found = True
                else:
                    self.get_logger().info('Nincs U alakú parkoló, keresek tovább...')
                    twist.angular.z = 0.5
            else:
                self.get_logger().info('Nincs right_points_len vagy entrance_points_len attribútum!')
                twist.angular.z = 0.5
        else:
            self.get_logger().info(f'Parkolás: min_front={min_front}')
            if min_front is not None and min_front > self.obstacle_threshold:
                twist.linear.x = 0.15
            else:
                self.get_logger().info('Beparkoltam!')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.target_found = False

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Küldött Twist: lin.x={twist.linear.x}, ang.z={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = ParkingLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()