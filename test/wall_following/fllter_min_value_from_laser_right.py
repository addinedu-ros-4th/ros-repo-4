#laser scan을 구독해서 얻은 값에서 오른쪽의 데이터를 가져오는 코드 구현


import rclpy as rp
from sensor_msgs.msg import LaserScan
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
        self.subscription

    def callback(self, msg):
        print("ranges: ")
        #데이터에서 0이거나 1 이상인 값을 제외한다

        temp_range = msg.ranges[50:91] #y
        print(len(temp_range))
        filtered_data = [value for value in temp_range if 0.0 < value < 1.0]
        print("min: ")
        print(len(filtered_data))

        #index를 x, laser 포인트를 y로 더글라스 포이커 알고리즘을 사용하여 직선 검출
        temp_value = list(range(0,len(filtered_data))) #x
        print("len temp_value")
        print(len(temp_value))

        combined_list = [[x, y] for x, y in zip(temp_value, filtered_data)]

        result = rdp(combined_list, epsilon=0.1)
        print("rdp result :",end="")
        print(result)

        # 기울기 계산
        m = (result[1][1] - result[0][1]) / (result[1][0] - result[0][0])


        # m,b = self.line_equation(result[0],result[1])
        print(" line m:", end="")
        print(m)


    
    def line_equation(p1, p2):
        print("in equation")
        x1, y1 = p1
        x2, y2 = p2
        
        if x1 == x2:
            raise ValueError("The two points have the same x value, resulting in a vertical line.")
        
        # 기울기 계산
        m = (y2 - y1) / (x2 - x1)
        
        # y절편 계산
        b = y1 - m * x1
        
        return m, b
        

def main(args=None):
    rp.init(args=args)

    wall_follower_subscriber = LaserScanSubscriber()
    rp.spin(wall_follower_subscriber)

    wall_follower_subscriber.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()

