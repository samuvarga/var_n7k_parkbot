import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class ParkingLogicNode(Node):
    def __init__(self):
        super().__init__('parking_logic_node')
        self.odom_sub = self.create_subscription(Odometry, '/model/turtlebot3/odometry', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/model/turtlebot3/scan', self.lidar_callback, 10)
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
        self.lidar_data = msg

    def park_logic(self):
        if self.current_pose is None or self.lidar_data is None:
            return

        # LIDAR: előre néző sugarak (középső 20 fok)
        ranges = self.lidar_data.ranges
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment
        num_ranges = len(ranges)
        center_idx = num_ranges // 2
        window = int(math.radians(20) / angle_increment)
        front_ranges = ranges[center_idx - window//2 : center_idx + window//2]

        # Szabad hely keresése
        min_front = min(front_ranges)
        self.get_logger().info(f'Legközelebbi akadály elöl: {min_front:.2f} m')

        twist = Twist()

        if not self.target_found:
            # Ha elöl legalább target_distance szabad, akkor ott a parkolóhely
            if min_front > self.target_distance:
                self.get_logger().info('Szabad parkolóhelyet találtam, odamegyek!')
                twist.linear.x = 0.3
                self.target_found = True
            else:
                # Ha akadály van, forduljon el
                twist.angular.z = 0.5
        else:
            # Ha már parkolóhelyen vagyunk, lassan előre, amíg közel nem érünk valamihez
            if min_front > self.obstacle_threshold:
                twist.linear.x = 0.15
            else:
                self.get_logger().info('Beparkoltam!')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.target_found = False  # újra kereshet parkolóhelyet, ha szeretnéd

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()