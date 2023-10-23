from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  kikusui_electronic_load_node = Node(
      package='kikusui_electronic_load',
      executable='service'
  )

  ld = LaunchDescription()
  ld.add_action(kikusui_electronic_load_node)

  return ld