import rclpy
from rclpy.node import Node
from nina_msgs.srv import AddTwoInts

class SimpleServiceServer(Node):
    
    def __init__(self):
        super().__init__("simple_service_server")

        self.service = self.create_service(AddTwoInts, "add_two_ints", self.serviceCallback)

        self.get_logger().info("Service add_two_ints Ready")

    def serviceCallback(self, req , res):
        self.get_logger().info("Two new reqs received: a = %d, b = %d" % (req.a,req.b) )
        res.sum = req.a + req.b
        self.get_logger().info("Sending back response: sum = %d" % res.sum)
        return res
def main():
    rclpy.init()
    simpleserviveserver = SimpleServiceServer()
    rclpy.spin(simpleserviveserver)
    simpleserviveserver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    