import rclpy 
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy import Parameter

class Simple_Parameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")
        self.declare_parameter("simple_int_param",14)
        self.declare_parameter("simple_string_param","Nina")

        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, param):
        result = SetParametersResult()

        for param in param:
            if param.name == "simple_int_param" and param.type == Parameter.Type.INTEGER:
                self.get_logger().info("Param simple_int_param is changed and the new param is:%d" %param.value)

                result.successful = True
            if param.name =="simple_string_param" and param.type == Parameter.Type.STRING:
                self.get_logger().info("Param simple_string_param is changed and the new param is:%s" %param.value)
                result.successful = True

        return result
    

def main():
    rclpy.init()
    simple_parm = Simple_Parameter()
    rclpy.spin(simple_parm)
    simple_parm.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

                
                



