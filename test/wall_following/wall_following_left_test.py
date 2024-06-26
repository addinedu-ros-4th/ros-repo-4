#laser scan을 구독해서 얻은 값으로 벽을 따라 주행하기 


import rclpy as rp
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rdp import rdp
from geometry_msgs.msg import Twist


right_laser_range_start = 50
right_laser_range_end = 90

left_laser_range_start = 180
left_laser_range_end = 220

front_laser_range_start = 110
front_laser_range_end =160

min_distance_to_wall = 0.1


class LaserScanSubscriber(Node):
    
    def __init__(self):
        super().__init__('wall_following')
        qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, qos)
        self.publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        
        self.subscription

    def callback(self, msg):

        #앞, 왼쪽, 오른쪽 범위를 나눠서 데이터 받기 
        front_raw_data = msg.ranges[front_laser_range_start:front_laser_range_end]
        left_raw_data = msg.ranges[left_laser_range_start:left_laser_range_end]
        right_raw_data = msg.ranges[right_laser_range_start:right_laser_range_end]

        # #데이터에서 0이거나 1 이상인 값을 제외한다

        front_filtered_data = [value for value in front_raw_data if 0.0 < value < 1.0]
        left_filtered_data = [value for value in left_raw_data if 0.0 < value < 1.0]
        right_filtered_data = [value for value in right_raw_data if 0.0 < value < 1.0]

        #data length만큼 x 데이터를 만든다

        temp_left_x = list(range(0,len(left_filtered_data)))
        temp_right_x = list(range(0,len(right_filtered_data)))

        combined_list_left = [[x,y] for x,y in zip(temp_left_x, left_filtered_data)]
        combined_list_right = [[x,y] for x,y in zip(temp_right_x, right_filtered_data)]

        #douglas-peucker 알고리즘을 사용하여 기울기를 구할 두 점을 검출 한다
        rdp_left_result = rdp(combined_list_left, epsilon=0.1)
        rdp_right_result = rdp(combined_list_right, epsilon=0.1)

        #기울기 계산
        
        a_left = -1 
        a_right = -1

        if len(rdp_left_result) == 2:
            a_left = ( rdp_left_result[1][1] - rdp_left_result[0][1]) /( rdp_left_result[1][0] - rdp_left_result[0][0]) 
            print("left_inclination : ", end="")
            print(a_left)
        else:
            print("cannot find inclination for LEFT side")

        if len(rdp_right_result) == 2:
            a_right = ( rdp_right_result[1][1] - rdp_right_result[0][1]) /( rdp_right_result[1][0] - rdp_right_result[0][0]) 
            print("right_inclination : ", end="")
            print(a_right)
        else:
            print("cannot find inclination for RIGHT side")

        #각 벽 까지의 거리 중 가장 min 한 값 구하기 


        if len(front_filtered_data) > 0:
            min_front = min(front_filtered_data)
            print("min_front :", end="")
            print(min_front)
        else:
            print("front_filtered_data is probably empty")

        if len(left_filtered_data) > 0 :
            min_left = min(left_filtered_data)
            print("left_filtered_data :", end="")
            print(min_left)
        else:
            print("left_filtered_data is probably empty")

        if len(right_filtered_data) > 0:
           min_right = min(right_filtered_data)
           print("right_filtered_data :", end="")
           print(min_right)
        else:
            print("right_filtered_data is probably empty")

        
        #왼쪽 벽만 생각하고 test 코드 작성 
        if a_left != -1 :
            if a_left < 0.001 and min_left > min_distance_to_wall :
                msg = Twist()
                msg.linear.x = 1.0
                msg.angular.z = 0.0
                self.publisher.publish(msg)
        else:
            print("no wall!")




            



def main(args=None):
    rp.init(args=args)

    wall_follower_subscriber = LaserScanSubscriber()
    rp.spin(wall_follower_subscriber)
    
    wall_follower_subscriber.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()

