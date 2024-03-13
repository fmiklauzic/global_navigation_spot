from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    
    node = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ['--x', '0.04', '--y', '0.0', '--z', '0.06', '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5', '--frame-id', 'front_rail', '--child-frame-id', 'camera_link'])
                       
    ld = LaunchDescription()
    
    node = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ['--x', '-0.185', '--y', '0.0', '--z', '0.31', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0', '--frame-id', 'front_rail', '--child-frame-id', 'gps_link'])
                       
    
    ld.add_action(node)

    return ld
