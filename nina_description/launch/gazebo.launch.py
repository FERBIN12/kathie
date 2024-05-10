from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from ament_index_python import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration 
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    nina_description = get_package_share_directory("nina_description")
    nina_description_prefix = get_package_prefix("nina_description")

    model_path = os.path.join(nina_description, "models")
    model_path += os.pathsep + os.path.join(nina_description_prefix, "share")

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)


    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value= os.path.join(nina_description, "urdf", "nina.urdf.xacro"),
        description = "Abs path of the model"
    )
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),value_type=str)

    robot_state_pubisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description" : robot_description}]
    )

    gz_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join
                (get_package_share_directory("gazebo_ros"),"launch", "gzserver.launch.py")))
    

    gz_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join
                (get_package_share_directory("gazebo_ros"),"launch", "gzclient.launch.py")))
    
    spawn_robot = Node(
        package= "gazebo_ros",
        executable = "spawn_entity.py",
        arguments = ["-entity", "Nina", "-topic", "robot_description"],
        output = "screen"
    )


    return LaunchDescription([
        env_var,
        model_arg,
        robot_state_pubisher,
        gz_server,
        gz_client,
        spawn_robot
    

    ])