import rclpy as rp
from turtlesim.srv import TeleportAbsolute

rp.init()
test_node = rp.create_node('client_test')

service_name = '/turtle1/teleport_absolute'
cli = test_node.create_client(TeleportAbsolute, service_name)
req = TeleportAbsolute.Request()
req.x = 7.
req.y = 7.
req.theta = 3.14

print(req)

while not cli.wait_for_service(timeout_sec=1.0):
    print("Waiting for service")

future = cli.call_async(req)

while not future.done():
    rp.spin_once(test_node)
    print(future.done(), future.result())