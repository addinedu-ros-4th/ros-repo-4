from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped

rclpy.init()
print("init")

nav = BasicNavigator()
print("instance")

init_pose = PoseStamped()
init_pose.header.frame_id = 'map'
init_pose.header.stamp = nav.get_clock().now().to_msg()
init_pose.pose.position.x = 1.2232815327112527
init_pose.pose.position.y = -0.10594246908756004
init_pose.pose.position.z = 0.0
init_pose.pose.orientation.x = 0.0
init_pose.pose.orientation.y = 0.0
init_pose.pose.orientation.z = 0.6286978029737526
init_pose.pose.orientation.w = 0.7776497106898302

nav.setInitialPose(init_pose)
print("initial pose")

nav.waitUntilNav2Active()
print("nav2active")




goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = nav.get_clock().now().to_msg()
goal_pose.pose.position.x = 4.2162089347839355
goal_pose.pose.position.y = 0.45503297448158264
goal_pose.pose.position.z = -0.001434326171875
goal_pose.pose.orientation.x = 0.0
goal_pose.pose.orientation.y = 0.0
goal_pose.pose.orientation.z = 0.0
goal_pose.pose.orientation.w = 1.0

print("posestamped")


nav.goToPose(goal_pose)
