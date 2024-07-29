import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from nina_msgs.srv import GetTransform  
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse

class SimpleTkKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")

        self.static_tf2_broadcaster_ = StaticTransformBroadcaster(self)  #for broadcasting transforms from usingt the tf2 lib
                                                                         # The supported message type in tf2 is Transformed stamped from geomrtry_msgs
        self.dynamic_tf2_broadcaster_ = TransformBroadcaster(self)
        
        
        self.static_transformed_stamped_ = TransformStamped()
        self.dynamic_transform_stamped_ = TransformStamped()

        self.x_increment_ = 0.05
        self.last_x_ = 0.0
        self.rotation_counter_ = 0
        self.last_orientation_ = quaternion_from_euler(0 ,0 ,0) #in-terms of radian 
        self.orientation_increment_ = quaternion_from_euler(0 ,0 ,0.05) 

        self.buffer_ = Buffer()
        self.transform_listener_ = TransformListener(self.buffer_, self)
        
        # we use the tf2 lib to receive the present timestamp using the get_clock from Node lib

        self.static_transformed_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transformed_stamped_.header.frame_id = "ninabot_base"
        self.static_transformed_stamped_.child_frame_id = "ninabot_top"

        #we set the translation transforms needed for the bot

        self.static_transformed_stamped_.transform.translation.x = 0.0
        self.static_transformed_stamped_.transform.translation.y = 0.0
        self.static_transformed_stamped_.transform.translation.z = 0.1

        # Now we define the rotation transforms 
        self.static_transformed_stamped_.transform.rotation.x = 0.0
        self.static_transformed_stamped_.transform.rotation.y = 0.0
        self.static_transformed_stamped_.transform.rotation.z = 0.0
        self.static_transformed_stamped_.transform.rotation.w = 1.0


        # Now we broadcast the created transformed messge using statictransformebroadcaster

        self.static_tf2_broadcaster_.sendTransform(self.static_transformed_stamped_)
        self.get_logger().info("publishing static transform between %s and %s" %
                               (self.static_transformed_stamped_.header.frame_id, self.static_transformed_stamped_.child_frame_id))
        
        self.timer_ = self.create_timer(0.1, self.timerCallback)

        self.get_transform_srv_ = self.create_service(GetTransform,"get_transform",self.transformCallback)

    def timerCallback(self):
        # similar approach is used for determining the dynamic tramsforms 

        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id = "ninabot_base"

        #we set the translation transforms needed for the bot

        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0   

        # we use quaternion function to update the orientation of the bot 
        
        q = quaternion_multiply(self.last_orientation_, self.orientation_increment_)

        # Now we define the rotation transforms 
        self.dynamic_transform_stamped_.transform.rotation.x = q[0]
        self.dynamic_transform_stamped_.transform.rotation.y = q[1]
        self.dynamic_transform_stamped_.transform.rotation.z = q[2]
        self.dynamic_transform_stamped_.transform.rotation.w = q[3]

        # Now we broadcast the created transformed messge using TransformBroadcaster
        self.dynamic_tf2_broadcaster_.sendTransform(self.dynamic_transform_stamped_)
        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x

        self.rotation_counter_ += 1
        self.last_orientation_ = q

        if self.rotation_counter_ > 100:
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)
            self.rotation_counter_ = 0

    def transformCallback(self, req, res):
        self.get_logger().info("Publishing transforms between %s and %s" % (req.frame_id, req.child_frame_id))
        requested_transform_ = TransformStamped()
        try:
            requested_transform_ = self.buffer_.lookup_transform(req.frame_id, req.child_frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error("Hey mate an error has occured when trying to transform between these frames a: %s and b: %s"  % (req.frame_id, req.child_frame_id))
            res.success = False
            return res
        
        res.transform = requested_transform_
        res.success = True
        return res
 

def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTkKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()    

if __name__ == "__main__":
    main()
        

