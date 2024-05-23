#laser scan을 구독해서 얻은 값으로 벽을 따라 주행하기 


import rclpy as rp
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rdp import rdp
from geometry_msgs.msg import Twist


right_laser_range_start = 60
right_laser_range_end = 80

diagonal_right_laser_range_start = 90
diagonal_right_laser_range_end = 110

front_laser_range_start = 120
front_laser_range_end =150

diagonal_left_laser_range_start = 160
diagonal_left_laser_range_end = 180

left_laser_range_start = 190
left_laser_range_end = 210



min_distance_to_wall = 0.2
min_inclination = 0.002
min_distance_for_front = 0.3


class WallFollowing(Node):
    
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
        diagonal_left_raw_data = msg.ranges[diagonal_left_laser_range_start: diagonal_left_laser_range_end]
        diagonal_right_raw_data = msg.ranges[diagonal_right_laser_range_start: diagonal_right_laser_range_end]

        # #데이터에서 0이거나 1 이상인 값을 제외한다

        front_filtered_data = [value for value in front_raw_data if 0.0 < value < 1.0]
        left_filtered_data = [value for value in left_raw_data if 0.0 < value < 1.0]
        right_filtered_data = [value for value in right_raw_data if 0.0 < value < 1.0]
        diagonal_left_filtered_data = [value for value in diagonal_left_raw_data if 0.0 < value < 1.0]
        diagonal_right_filtered_data = [value for value in diagonal_right_raw_data if 0.0 < value < 1.0]

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

        #min value 가 -1이면 filtered 가 비어 있다는 얘기, 다 0 이거나 다 멀리 있거나 
        min_front = 0
        min_left = 0
        min_right = 0
        min_diagonal_left = 0
        min_diagonal_right = 0 
        
        if len(front_filtered_data) > 0:
            min_front = min(front_filtered_data)
            print("min_front :", end="")
            print(min_front)
        else:
            print("front_filtered_data is probably empty")

        if len(left_filtered_data) > 0 :
            min_left = min(left_filtered_data)
            print("min_left :", end="")
            print(min_left)
        else:
            print("left_filtered_data is probably empty")

        if len(right_filtered_data) > 0:
           min_right = min(right_filtered_data)
           print("min_right :", end="")
           print(min_right)
        else:
            print("right_filtered_data is probably empty")

        if len(diagonal_left_filtered_data) > 0:
            min_diagonal_left = min(diagonal_left_filtered_data)
            print("min_diagonal_left :", end="")
            print(min_diagonal_left)
        else:
            print("diagonal_left_filtered_data is probably empty")

        if len(diagonal_right_filtered_data) > 0:
            min_diagonal_right = min(diagonal_right_filtered_data)
            print("min_diagonal_right: " , end="")
            print(min_diagonal_right)
        else:
            print("diagonal_right_filtered_data is probably empty")

        msg = Twist()

        if ((abs(a_left)< min_inclination)) and ((abs(a_right) < min_inclination)):
            print("wall is parallel")
            if (min_left > min_distance_to_wall) and (min_right > min_distance_to_wall) :
                print("move forward")
                msg.linear.x = 0.3
                msg.angular.z = 0.0

            elif min_left <= min_distance_to_wall :
                print("move diagonal right")
                msg.linear.x = 0.1
                msg.angular.z = -0.3

            elif min_right <= min_distance_to_wall:
                print("move diagonal left")
                msg.linear.x = 0.1
                msg.angular.z = 0.3

            else:
                print("I don't know what is happening 1 ")

            if ((a_left > 0) and (a_right > 0)) and ((a_right != -1) and (a_right < 0)):
                print("turn right")
                msg.linear.x = 0.0
                msg.angular.z = -0.3
            elif ((a_left != -1) and (a_left < 0)) and ((a_right != -1) and (a_right < 0)):
                print("turn left")
                msg.linear.x = 0.0
                msg.angular.z = 0.3
            else:
                print("unknown sign")

        else:
            if ((abs(a_left)>= min_inclination)) and ((abs(a_right) >= min_inclination)):
                print("both wall out of range")
                # if ((min_front == 0) or (min_front > min_distance_for_front)):
                if (min_diagonal_left> min_diagonal_right) and (min_diagonal_left > min_distance_to_wall) :
                    print("wall is far away.. left diagonal")
                    msg.linear.x = 0.1
                    msg.angular.z = 0.3
                elif (min_diagonal_right > min_diagonal_left) and (min_diagonal_right > min_distance_to_wall):
                    print("move forward..wall is far away.. right diagonal")
                    msg.linear.x = 0.1
                    msg.angular.z = -0.3
                else:
                    if ((min_front == 0) or (min_front > min_distance_for_front)):
                        print("move forward..wall is far away..")
                        msg.linear.x = 0.3
                        msg.angular.z = 0.0
                # else:
                #     if (min_diagonal_right > min_diagonal_left):
                #         print("min_front exist.. right diagonal")
                #         msg.linear.x = 0.1
                #         msg.angular.z = -0.2
                #     else:
                #         print("min_front exist..wall is far away..")
                #         msg.linear.x = 0.1
                #         msg.angular.z = 0.2

                
            elif abs(a_left) >= min_inclination:
                print("left wall inclination is out of range..try turning left")
                msg.linear.x = 0.0
                msg.angular.z = 0.2
            elif abs(a_right) >=min_inclination :
                print("right wall is out of range... try turning right")
                msg.linear.x = 0.0
                msg.angular.z = -0.2
            else:
                print("inclination error")
            
            


        print("-----------------")

        self.publisher.publish(msg)







def main(args=None):
    rp.init(args=args)

    wall_follower = WallFollowing()
    rp.spin(wall_follower)
    
    wall_follower.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()

