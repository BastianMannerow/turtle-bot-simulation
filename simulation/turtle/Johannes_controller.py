from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.action import DriveDistance, RotateAngle
import rclpy

class JohannesController(Node):
    """
    Controller for the Johannes turtle agent.

    This class defines the behavior of the Johannes turtle in the simulation.
    """

    def __init__(self, agent_list, environment):
        if not rclpy.ok():
            rclpy.init()
        super().__init__('johannes_controller_node')
        # e.g., store the names of different agents, if needed
        self.turtle_bots = {} # {agent_id: turtle_bot_object}
        # Subscriber for bumper
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_callback, qos_profile_sensor_data)
        # Undock action client (ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" # undock the turtle bot before moving)
        #self.undock_action_client = ActionClient(self, Undock, '/undock')
        # Rotate action client
        self.rotate_action_client = ActionClient(self, RotateAngle, '/tb1/rotate_angle')
        # Drive distance action client
        self.drive_distance_action_client = ActionClient(self, DriveDistance, '/tb1/drive_distance')
        print("Controller initialized")

        self.environment = environment
        self.agent_list = agent_list

        self.get_logger().info("JohannesController node has been started.")

        #tuple_of_agents_and_robot_id_or_dictionary = agent_list # kein tuple, sondern triple aus robot id agent id und temp der positionsdaten um bei bump rückgängig

    # https://github.com/iRobotEducation/irobot_create_msgs/blob/rolling/msg/HazardDetection.msg
    def hazard_callback(self, msg):
        for hazard in msg.detections:
            if hazard.type == 1 and 'bump' in hazard.header.frame_id:
                self.get_logger().warn(f"Bumper triggered: {hazard.header.frame_id}")
                self.environment.register_bumping(None)
    
    '''def undock_robot(self):
        self.get_logger().info("Starting undock action...")
        if not self.undock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Undock action server not available!")
            return False
        goal_msg = Undock.Goal()
        self.undock_action_client.send_goal_async(goal_msg)'''
    
    def _send_rotate_then_drive(self, angle, return_after=False):
        goal = RotateAngle.Goal()
        goal.angle = angle
        goal.max_rotation_speed = 1.0
        self.rotate_action_client.wait_for_server()
        send_goal_future = self.rotate_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(lambda f: self._rotate_goal_response_callback(f, angle, return_after))

    def _rotate_goal_response_callback(self, future, angle, return_after):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Rotate goal was rejected.')
            self.environment.robot_reached_position()
            return

        self.get_logger().info(f'Rotate goal accepted (angle={angle}).')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._rotate_result_callback(f, angle, return_after))

    def _rotate_result_callback(self, future, angle, return_after):
        result = future.result().result
        self.get_logger().info(f'Rotation complete (angle={angle}).')

        if return_after and abs(angle) == 1.57:  
            # rotate right first -> then forward, then rotate back
            self.get_logger().info("Rotation complete: moving forward before rotating back.")
            self._send_drive_goal(0.3, after_rotate_back=True, reverse_angle=-angle)
        else:
            # Normal rotation, move forward only if not part of a 'rotate back' step
            self._send_drive_goal(0.3)

    def _send_drive_goal(self, distance, after_rotate_back=False, reverse_angle=None):
        goal = DriveDistance.Goal()
        goal.distance = distance
        goal.max_translation_speed = 0.2
        self.drive_distance_action_client.wait_for_server()
        send_goal_future = self.drive_distance_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda f: self._drive_goal_response_callback(f, after_rotate_back, reverse_angle)
        )

    def _drive_goal_response_callback(self, future, after_rotate_back, reverse_angle):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Drive goal rejected :(')
            return

        self.get_logger().info('Drive goal accepted :)')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f: self._drive_result_callback(f, after_rotate_back, reverse_angle)
        )

    def _drive_result_callback(self, future, after_rotate_back, reverse_angle):
        result = future.result().result
        self.get_logger().info('Drive result received.')

        if after_rotate_back and reverse_angle is not None:
            self.get_logger().info(f"Driving done — rotating back by {reverse_angle} radians.")
            self._send_only_rotate(reverse_angle)
        else:
            self.environment.robot_reached_position()

    def _send_only_rotate(self, angle):
        goal = RotateAngle.Goal()
        goal.angle = angle
        goal.max_rotation_speed = 1.0
        self.rotate_action_client.wait_for_server()
        send_goal_future = self.rotate_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(lambda f: self._only_rotate_goal_response_callback(f))

    def _only_rotate_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Rotate-back goal was rejected.')
            self.environment.robot_reached_position()
            return

        self.get_logger().info('Rotate-back goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._only_rotate_result_callback(f))

    def _only_rotate_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Rotate-back complete.')
        self.environment.robot_reached_position()

    def return_robot_id_by_agent(self, agent):
        self.tuple_of_agents_and_robot_id_or_dictionary
        return None

# ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "distance: 0.3" # move forward 30 cm
# ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "angle: 1.558" #(pi/2) (realitygap: angle: 1.57) 90 degrees in radians to the left
# ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "angle: -1.558" #-(pi/2) (realitygap: angle: -1.57) 90 degrees in radians to the right
# ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "angle: 3.14" #(pi) 180 degrees in radians to the left
    def move_top(self, agent):
        print("Move agent top called")
        self.environment.robot_is_moving()
        #robot_id = jc.return_robot_id_by_agent(agent)
        self._send_drive_goal(0.3)

    def move_left(self, agent):
        print("Move agent left called")
        self.environment.robot_is_moving()
        self._send_rotate_then_drive(1.57, return_after=True)

    def move_right(self, agent):
        print("Move agent right called")
        self.environment.robot_is_moving()
        self._send_rotate_then_drive(-1.57, return_after=True)

    def move_bottom(self, agent):
        print("Move agent bottom called")
        self.environment.robot_is_moving()
        self._send_drive_goal(-0.3)