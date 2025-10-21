from typing import Any
import rclpy
from rclpy.node import Node
from irobot_create_msgs.action import Undock
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import HazardDetectionVector
from turtlesim.action import RotateAngle
from turtlesim.action import DriveDistance
from simulation.Environment import move_agent_top, move_agent_bottom, move_agent_left, move_agent_right, register_bumping

class JohannesController(Node):
    """
    Controller for the Johannes turtle agent.

    This class defines the behavior of the Johannes turtle in the simulation.
    """

    def __init__(self):
        super().__init__('johannes_controller_node')
        # e.g., store the names of different agents, if needed
        self.turtle_bots = {} # {agent_id: turtle_bot_object}
        # Subscriber for bumper
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_callback, qos_profile_sensor_data)
        # Undock action client
        #self.undock_action_client = ActionClient(self, Undock, '/undock')
        # Rotate action client
        self.rotate_action_client = ActionClient(self, RotateAngle, '/rotate_angle')
        # Drive distance action client
        self.drive_distance_action_client = ActionClient(self, DriveDistance, '/drive_distance')
        print("Controller initialized")
    
    

        # ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" # undock the turtle bot before moving
        # ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "distance: 0.3" # move forward 30 cm
        # ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "angle: 1.558" #(pi/2) 90 degrees in radians to the left
        # ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "angle: -1.558" #-(pi/2) 90 degrees in radians to the right
        # ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "angle: 3.14" #(pi) 180 degrees in radians to the left
        
    def hazard_callback(self, msg: HazardDetectionVector):
        for hazard in msg.detections:
            if hazard.type == 1 and hazard.header.frame_id == "bump_right" or hazard.header.frame_id == "bump_front_right" or hazard.header.frame_id == "bump_front_center" and self.turning == 0:
                print("Right bumper pressed.")
                return register_bumping()
            elif hazard.type == 1 and hazard.header.frame_id == "bump_left" or hazard.header.frame_id == "bump_front_left" and self.turning == 0:
                print("Left bumper pressed.")
                return register_bumping()
    
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
        self.rotate_action_client.send_goal_async(goal_msg, feedback_callback=self.rotate_feedback_callback)
        self._send_goal_future.add_done_callback(self.rotate_response_callback)

    def send_drive_distance_goal(self, distance: float):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = 0.2  # Set a maximum translation speed
        self.drive_distance_action_client.wait_for_server()
        self.drive_distance_action_client.send_goal_async(goal_msg, feedback_callback=self.drive_feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: {result}')
        rclpy.shutdown()

    def main(args=None, agent_id=None):
        rclpy.init(args=args)
        node = JohannesController()
        if move_agent_top():
            ActionClient.send_goal_async(node, DriveDistance, '/drive_distance', distance=0.3)
        elif move_agent_left():
            ActionClient.send_goal_async(node, RotateAngle, '/rotate_angle', angle=1.57)
            rclpy.spin(node)
            ActionClient.send_goal_async(node, DriveDistance, '/drive_distance', distance=0.3)
        elif move_agent_right():
            ActionClient.send_goal_async(node, RotateAngle, '/rotate_angle', angle=-1.57)
            rclpy.spin(node)
            ActionClient.send_goal_async(node, DriveDistance, '/drive_distance', distance=0.3)
        elif move_agent_bottom():
            ActionClient.send_goal_async(node, RotateAngle, '/rotate_angle', angle=3.14)
            rclpy.spin(node)
            ActionClient.send_goal_async(node, DriveDistance, '/drive_distance', distance=0.3)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()
            