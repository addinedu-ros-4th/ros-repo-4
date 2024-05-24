

import rclpy as rp
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from rdp import rdp

class LaserScanSubscriber(Node):
    
    def __init__(self):
        super().__init__('wall_following')
        qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, qos)
        self.publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', qos)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription
    
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def callback(self, msg):
        print("ranges: ")
        

def main(args=None):
    rp.init(args=args)

    wall_follower_subscriber = LaserScanSubscriber()
    rp.spin(wall_follower_subscriber)

    wall_follower_subscriber.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()

