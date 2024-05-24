#laser_scan msg 토픽 subscribe 테스트 

import rclpy as rp
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class LaserScanSubscriber(Node):
    
    def __init__(self):
        super().__init__('wall_following')
        qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, qos)
        self.subscription

    def callback(self, msg):
        print("ranges: ")
        for idx,val in enumerate(msg.ranges):
            print(idx,end="")
            print(":")
            print(val)

def main(args=None):
    rp.init(args=args)

    wall_follower_subscriber = LaserScanSubscriber()
    rp.spin(wall_follower_subscriber)

    wall_follower_subscriber.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()

