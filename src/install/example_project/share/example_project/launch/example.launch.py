from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '/home/supreme/example_project_ws/src/example_project/example_project/customer_v4.py', '1'],
            name='table1'
        ),
        ExecuteProcess(
            cmd=['python3', '/home/supreme/example_project_ws/src/example_project/example_project/customer_v4.py', '2'],
            name='table2'
        ),
        ExecuteProcess(
            cmd=['python3', '/home/supreme/example_project_ws/src/example_project/example_project/customer_v4.py', '3'],
            name='table3'
        ),
        ExecuteProcess(
            cmd=['python3', '/home/supreme/example_project_ws/src/example_project/example_project/customer_v4.py', '4'],
            name='table4'
        ),
        ExecuteProcess(
            cmd=['python3', '/home/supreme/example_project_ws/src/example_project/example_project/kitchen_v4.py'],
            name='kitchen'
        ),
    ])