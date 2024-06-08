import rclpy as rp
from rclpy.action import ActionServer
from rclpy.node import Node

from test_package_msgs.action import CountNumber
import time

class CountingServer(Node):
    def __init__(self):
        super().__init__('counting_action')
        self.action_server = ActionServer(self, CountNumber, 'counting_server', self.execute_callback)


    def execute_callback(self, goal_handle):
        
        feedback_msg = CountNumber.Feedback()
        for n in range(0,goal_handle.request.request_num):
            temp_num = goal_handle.request.request_num
            feedback_msg.current_num = temp_num - n
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        goal_handle.succeed()
        result = CountNumber.Result()
        result.result = "action server SUCCEED"
        return result
    

def main(args=None):
    rp.init(args=args)
    counting_server = CountingServer()
    rp.spin(counting_server)


if __name__ == '__main__':
    main()