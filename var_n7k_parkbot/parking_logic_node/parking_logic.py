#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math

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
        # self.get_logger().info(f'ODOM: {self.current_pose}')

    def lidar_callback(self, msg):
        min_front = float('inf')
        min_left = float('inf')
        min_right = float('inf')
        min_right_front = float('inf')
        min_right_side = float('inf')
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            # Szűrd ki a robot testének dobozát
            if -0.5 < x < 0.5 and -0.5 < y < 0.5:
                continue
            # Csak a -0.2-nél nagyobb z értékeket engedd át
            if z < -0.2:
                continue
            # Előre néző sáv
            if x > 0 and abs(y) < 0.2:
                min_front = min(min_front, x)
            # Bal oldal (kb. 90 fok)
            if abs(x) < 0.3 and y > 0.2:
                min_left = min(min_left, y)
            # Jobb oldal (kb. -90 fok)
            if abs(x) < 0.3 and y < -0.2:
                min_right = min(min_right, abs(y))
            # Jobbra előre (parkoló bejárat)
            if y < -0.5 and abs(x) < 0.3:
                min_right_side = min(min_right_side, abs(y))
            if x > 0.5 and y < -0.5 and x < 1.5:
                min_right_front = min(min_right_front, x)
        self.lidar_data = (
            min_front if min_front != float('inf') else None,
            min_left if min_left != float('inf') else None,
            min_right if min_right != float('inf') else None,
            min_right_front if min_right_front != float('inf') else None,
            min_right_side if min_right_side != float('inf') else None
        )
        # self.get_logger().info(f'LIDAR min_front: {self.lidar_data[0]}, min_left: {self.lidar_data[1]}, min_right: {self.lidar_data[2]}')
        print("LIDAR:", self.lidar_data)

    def park_logic(self):
        if self.current_pose is None or self.lidar_data is None:
            return

        min_front, min_left, min_right, min_right_front, min_right_side = self.lidar_data
        twist = Twist()

        if not self.target_found:
            # Csak jobbra keresünk U alakú parkolóhelyet
            if min_right_side is not None and min_right_side > self.target_distance:
                self.get_logger().info('Jobbra U alakú parkolóhelyet találtam, odafordulok!')
                twist.angular.z = -0.5
                self.target_found = True
            else:
                twist.angular.z = 0.5  # Keres tovább
        else:
            # Parkolás előre, amíg akadály nincs előtte
            if min_front is not None and min_front > self.obstacle_threshold:
                twist.linear.x = 0.15
            else:
                self.get_logger().info('Beparkoltam!')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.target_found = False

        self.cmd_vel_pub.publish(twist)
        # self.get_logger().info(f'Küldött Twist: lin.x={twist.linear.x}, ang.z={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = ParkingLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()