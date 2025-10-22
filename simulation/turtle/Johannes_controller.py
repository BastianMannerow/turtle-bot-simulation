from typing import Any
from rclpy.node import Node
from irobot_create_msgs.action import Undock
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveDistance
import time

class JohannesController(Node):
    """
    Controller for the Johannes turtle agent.

    This class defines the behavior of the Johannes turtle in the simulation.
    """

    def __init__(self, agent_list, environment):
        super().__init__('johannes_controller_node')
        # e.g., store the names of different agents, if needed
        self.turtle_bots = {} # {agent_id: turtle_bot_object}
        # Subscriber for bumper
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_callback, qos_profile_sensor_data)
        # Undock action client (ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" # undock the turtle bot before moving)
        #self.undock_action_client = ActionClient(self, Undock, '/undock')
        # Rotate action client
        self.rotate_action_client = ActionClient(self, RotateAngle, '/rotate_angle')
        # Drive distance action client
        self.drive_distance_action_client = ActionClient(self, DriveDistance, '/drive_distance')
        print("Controller initialized")

        self.environment = environment

        tuple_of_agents_and_robot_id_or_dictionary = agent_list

    # https://github.com/iRobotEducation/irobot_create_msgs/blob/rolling/msg/HazardDetection.msg
    def hazard_callback(self, msg: HazardDetectionVector):
        for hazard in msg.detections:
            if hazard.type == 1 and hazard.header.frame_id == "bump_right" or hazard.header.frame_id == "bump_front_right" or hazard.header.frame_id == "bump_front_center" and self.turning == 0:
                print("Right bumper pressed.")
                return self.environment.register_bumping()
            elif hazard.type == 1 and hazard.header.frame_id == "bump_left" or hazard.header.frame_id == "bump_front_left" and self.turning == 0:
                print("Left bumper pressed.")
                return self.environment.register_bumping()
    
    '''def undock_robot(self):
        self.get_logger().info("Starting undock action...")
        if not self.undock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Undock action server not available!")
            return False
        goal_msg = Undock.Goal()
        self.undock_action_client.send_goal_async(goal_msg)'''
    
    def send_rotate_goal(self, angle_degree: float):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle_degree
        goal_msg.max_rotation_speed = 1.0  # Set a maximum rotation speed
        self.rotate_action_client.wait_for_server()
        self.rotate_action_client.send_goal(goal_msg)

    def send_drive_distance_goal(self, distance: float):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = 0.2  # Set a maximum translation speed
        self.drive_distance_action_client.wait_for_server()
        self.drive_distance_action_client.send_goal(goal_msg)

    def return_robot_id_by_agent(self, agent):
        self.tuple_of_agents_and_robot_id_or_dictionary
        return None

# ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "distance: 0.3" # move forward 30 cm
# ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "angle: 1.558" #(pi/2) (realitygap: angle: 1.57) 90 degrees in radians to the left
# ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "angle: -1.558" #-(pi/2) (realitygap: angle: -1.57) 90 degrees in radians to the right
# ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "angle: 3.14" #(pi) 180 degrees in radians to the left
def move_top(agent, jc):
    robot_id = jc.return_robot_id_by_agent(agent)
    node = JohannesController()
    node.send_drive_distance_goal(0.3)

def move_left(agent, jc):
    node = JohannesController()
    node.send_rotate_goal(1.57)
    time.sleep(3)
    node.send_drive_distance_goal(0.3)

def move_right(agent, jc):
    node = JohannesController()
    node.send_rotate_goal(-1.57)
    time.sleep(3)
    node.send_drive_distance_goal(0.3)

def move_bottom(agent, jc):
    node = JohannesController()
    node.send_rotate_goal(3.14)
    time.sleep(5)
    node.send_drive_distance_goal(0.3)

