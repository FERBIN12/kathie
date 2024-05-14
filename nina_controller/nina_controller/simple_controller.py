#/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray 
from geometry_msgs.msg import TwistStamped
import numpy as np

class SimpleController(Node):
    def __init__(self):
        super().__init__("simplecontroller")

        self.declare_parameter("wheel_radius",0.033)
        self.declare_parameter("wheel_seperation",0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_seperation_ = self.get_parameter("wheel_seperation").get_parameter_value().double_value

        self.get_logger().info("Using wheel_radius %f" %self.wheel_radius_)
        self.get_logger().info("Using wheel_seperation %f" %self.wheel_seperation_)

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray,"simple_velocity_controller/commands",10)
        self.vel_pub_ = self.create_subscription(TwistStamped,"nina_controller/cmd_vel",self.velcallback ,10)

        #we use the matrix conversion needed for calculating speed.....
        self.speed_conversion_ = np.array([[self.wheel_radius_/2,self.wheel_radius_/2],
                                           [self.wheel_radius_ / self.wheel_seperation_, -self.wheel_radius_/ self.wheel_seperation_]]) 
        
        self.get_logger().info("The conversion matrix is: %s" %self.speed_conversion_)
 

    def velcallback(self,msg):

        robot_speed = np.array([[msg.linear.x],
                                [msg.linear.z]])
        
        wheel_speed  = np.matmul(np.linalg.inv(self.speed_conversion_),robot_speed)

        # then in the wheel speed we define the wheel speed seperately b4 that the data is converted into array

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[0,1],wheel_speed[0,0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def main():
        rclpy.init()
        simple_controller = SimpleController()
        rclpy.spin(simple_controller)
        simple_controller.destroy_node()
        rclpy.shutdown()
    if __name__ == "__main__":
        main()


