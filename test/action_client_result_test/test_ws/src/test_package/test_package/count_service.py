from test_package_msgs.srv import CountServiceMsg
from test_package_msgs.action import CountNumber
from test_package_msgs.msg import ActionClientResult

import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionClient
import time

class CountService(Node):

    def __init__(self):
        super().__init__('count_service_node')
        self.server = self.create_service(CountServiceMsg, 'count_service', self.callback_service)
        self.action_client = ActionClient(self,CountNumber, 'counting_server')
        self.publisher = self.create_publisher(ActionClientResult, 'pub_count',10)


        self.wait_for_result = False
    
    def callback_service(self,request, response):
        print('Request : ', request)
        
        temp_num = request.num                          #action에 넘겨줄 숫자 받음
        goal_msg = CountNumber.Goal()                   #action의 request 를 인스턴스 
        goal_msg.request_num = temp_num

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)


        response.result = True
        return response
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected : (')
            return
        
        self.get_logger().info('Goal accepted :)')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Action Server Result: {0}'.format(result.result))
        msg = ActionClientResult()
        msg.result = "action server :) result"
        self.publisher.publish(msg)
        self.wait_for_result = True

    
    def feedback_callback(self,feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback : {0}'.format(feedback.current_num))
    

def main(args=None):
    rp.init(args=args)
    count_service = CountService()
    rp.spin(count_service)
    rp.shutdown()


if __name__ == '__main__':
    main()