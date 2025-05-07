#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np
import sklearn.cluster
from visualization_msgs.msg import Marker, MarkerArray
import warnings

warnings.filterwarnings("ignore", category=RuntimeWarning)

class ParkingLogicNode(Node):
    def __init__(self):
        print("KONSTRUKTOR ELEJE")
        super().__init__('parking_logic_node')
        self.odom_sub = self.create_subscription(Odometry, '/model/turtlebot3/odometry', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/model/turtlebot3/scan/points', self.lidar_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/model/turtlebot3/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/model/turtlebot3/parking_markers', 10)

        self.current_pose = None
        self.lidar_data = None
        #teszt timer
        #self.test_timer = self.create_timer(0.5, lambda: self.get_logger().info('TIMER TESZT FUT!'))

        self.parking_detected = False
        self.turning_timer = None

        self.get_logger().info('ParkingLogicNode init kész, timer elindítva')
        self.get_logger().info('Timer létrejött, park_logic timer callback beállítva')
        # self.get_logger().info(f'TIMER OBJEKTUM: {self.timer}')

        self.lidar_callback_count = 0

        # MINDEN inicializáció után:
        self.park_logic_timer = self.create_timer(0.2, self.park_logic)
        print("KONSTRUKTOR VÉGE", self.park_logic_timer, self.park_logic)
 
        # --- TESZT: robot folyamatosan forogjon indulás után ---
        # twist = Twist()
        # twist.linear.x = 0.0
        # twist.angular.z = 0.5
        # self.cmd_vel_pub.publish(twist)
        # self.get_logger().info('TESZT: Küldtem egy folyamatos forgás parancsot!')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        #self.get_logger().info(f'odom_callback: {self.current_pose}')

    def lidar_callback(self, msg):
        self.lidar_callback_count += 1
        if self.lidar_callback_count < 5:
            return
        points = []
        # self.get_logger().info('lidar_callback hívva')  # <-- EZT IS KOMMENTELD KI
        # self.get_logger().info(f'Pontok száma szűrés előtt: {len(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))}')
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if (-0.4 < x < 0.4) and (-0.4 < y < 0.4):
                continue
            if z < -0.2:
                continue
            points.append((x, y, z))
        if not points:
            self.lidar_data = (None, None, None, None, None)
            # self.get_logger().warn('Nincsenek LIDAR pontok!')
            return

        points = np.array(points)
        # self.get_logger().info(f'LIDAR pontok száma: {len(points)}')

        # Szűrt tömbök
        front_mask = (points[:,0] > 0) & (np.abs(points[:,1]) < 0.05)  # 10 cm sáv
        left_mask = (points[:,1] > 0.5) & (np.abs(points[:,0]) < 0.5)
        right_mask = (points[:,1] < -0.5) & (np.abs(points[:,0]) < 0.5)
        right_front_mask = (points[:,0] > 0.5) & (points[:,1] < -0.5)
        right_side_mask = (points[:,1] < -0.5) & (np.abs(points[:,0]) < 0.5)

        def safe_min(arr, axis=0, default=10.0):
            return np.min(arr, axis=axis) if arr.size else default

        min_front = safe_min(points[front_mask, 0])
        min_left = safe_min(points[left_mask, 1])
        min_right = safe_min(points[right_mask, 1])
        min_right_front = safe_min(points[right_front_mask, 0])
        min_right_side = safe_min(points[right_side_mask, 1])

        self.lidar_data = (min_front, min_left, min_right, min_right_front, min_right_side)

        # Csak akkor detektáljon parkolót, ha tényleg van elég LIDAR adat ÉS azok szórása is életszerű!
        # if len(points) < 100 or np.std(points[:,0]) < 0.2 or np.std(points[:,1]) < 0.2:
        #     return

        # U-alakú parkoló detektálásához szükséges értékek eltárolása
        right_points = points[(points[:,1] < -0.6) & (np.abs(points[:,0]) < 0.6)]
        entrance_points = points[(points[:,0] > 0.6) & (np.abs(points[:,1]) < 0.6)]
        back_points = points[(points[:,0] < -0.4) & (np.abs(points[:,1]) < 0.6)]
        self.right_points_len = len(right_points)
        self.entrance_points_len = len(entrance_points)
        self.back_points_len = len(back_points)

        if len(right_points) > 0:
            # DBSCAN klaszterezés (eps=0.2m, min_samples=5)
            clustering = sklearn.cluster.DBSCAN(eps=0.2, min_samples=5).fit(right_points[:, :2])
            labels = clustering.labels_
            unique_labels = set(labels)
            marker_array = MarkerArray()

            max_cluster_size = 0
            target_centroid = None

            for cluster_id in unique_labels:
                if cluster_id == -1:
                    continue  # noise
                cluster_points = right_points[labels == cluster_id]
                # Robusztus szűrés minden problémás esetre
                if (
                    cluster_points is None
                    or cluster_points.size == 0
                    or not np.isfinite(cluster_points).all()
                    or np.isnan(cluster_points).any()
                    or np.isinf(cluster_points).any()
                    or np.any(np.abs(cluster_points) > 1e3)  # extrém outlierek kizárása
                ):
                    continue
                centroid_xy = np.mean(cluster_points[:, :2].astype(np.float64), axis=0)
                centroid = np.array([centroid_xy[0], centroid_xy[1], 0.1])
                # Zöld gömb a centroidhoz (marad)
                marker = Marker()
                marker.header.frame_id = "turtlebot3/base_footprint"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "parking_spots"
                marker.id = int(cluster_id)
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(centroid[0])
                marker.pose.position.y = float(centroid[1])
                marker.pose.position.z = float(centroid[2])
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker_array.markers.append(marker)

                # cyan kis gömbök a klaszter pontjaira
                # for j, pt in enumerate(cluster_points):
                #     pt_marker = Marker()
                #     pt_marker.header.frame_id = "turtlebot3/base_footprint"
                #     pt_marker.header.stamp = self.get_clock().now().to_msg()
                #     pt_marker.ns = f"cluster_points_{cluster_id}"
                #     pt_marker.id = j
                #     pt_marker.type = Marker.SPHERE
                #     pt_marker.action = Marker.ADD
                #     pt_marker.pose.position.x = float(pt[0])
                #     pt_marker.pose.position.y = float(pt[1])
                #     pt_marker.pose.position.z = 0.05
                #     pt_marker.scale.x = 0.04
                #     pt_marker.scale.y = 0.04
                #     pt_marker.scale.z = 0.04
                #     pt_marker.color.a = 1.0
                #     pt_marker.color.r = 0.0
                #     pt_marker.color.g = 1.0
                #     pt_marker.color.b = 1.0
                #     marker_array.markers.append(pt_marker)
                if cluster_points.shape[0] > max_cluster_size:
                    max_cluster_size = cluster_points.shape[0]
                    target_centroid = centroid

            self.marker_pub.publish(marker_array)
            if target_centroid is not None:
                self.parking_target = target_centroid
                #self.get_logger().info(f'parking_target beállítva: {self.parking_target}')
            else:
                self.get_logger().info('NINCS érvényes target_centroid!')

    def timer_test(self):
        self.get_logger().info("TIMER TEST FUT!")

    def park_logic(self):
        print("PARK_LOGIC FUT", self.current_pose, getattr(self, "parking_target", None))
        try:
            self.get_logger().info('PARK_LOGIC TIMER FUT!')
            self.get_logger().info(f'park_logic: entering, parking_target={getattr(self, "parking_target", None)}, current_pose={self.current_pose}')
            if not hasattr(self, 'parking_target') or self.parking_target is None or self.current_pose is None:
                self.get_logger().info('park_logic: return, missing data!')
                return

            dx = self.parking_target[0] - self.current_pose.position.x
            dy = self.parking_target[1] - self.current_pose.position.y
            distance = math.hypot(dx, dy)
            angle_to_target = math.atan2(dy, dx)
            # Quaternion -> yaw
            q = self.current_pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            angle_diff = angle_to_target - yaw

            twist = Twist()
            if abs(angle_diff) > 0.1:
                twist.angular.z = 0.5 * np.sign(angle_diff)
                twist.linear.x = 0.0
            elif distance > 0.1:
                twist.angular.z = 0.0
                twist.linear.x = 0.15
            else:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.get_logger().info('Centroidhoz beállt!')
            self.get_logger().info(f'park_logic: parking_target={self.parking_target}, current_pose={self.current_pose}')
            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f'park_logic exception: {e}')
        self.get_logger().info(f'park_logic_timer: {self.park_logic_timer}, park_logic: {self.park_logic}')

def main(args=None):
    rclpy.init(args=args)
    node = ParkingLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
