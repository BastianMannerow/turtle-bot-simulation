from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.action import DriveDistance, RotateAngle
import rclpy

class JohannesController(Node):
    """
    Controller for multiple TurtleBots in the simulation.
    Each simulation agent is mapped to one TurtleBot (tb1–tb4).
    """

    def __init__(self, agent_list, environment):
        if not rclpy.ok():
            rclpy.init()
        super().__init__('johannes_controller_node')

        self.environment = environment
        self.agent_list = agent_list
        self.turtle_bots = {}  # {agent: {"name": "tb1", "drive": client, "rotate": client}}

        # Map each agent to a TurtleBot name (tb1, tb2, tb3, tb4)
        turtle_names = ["tb1", "tb2", "tb3", "tb4"] # Rename to what you want, agents are mapped like: agent_list[0] -> turtle_names[0], etc. (reallife turtlebots have to be named accordingly over ros turtlebot-setup)
        for i, agent in enumerate(agent_list):
            if i < len(turtle_names):
                tb_name = turtle_names[i]
                self.turtle_bots[agent] = {
                    "name": tb_name,
                    "drive": ActionClient(self, DriveDistance, f"/{tb_name}/drive_distance"),
                    "rotate": ActionClient(self, RotateAngle, f"/{tb_name}/rotate_angle")
                }
                self.get_logger().info(f"Mapped agent {agent.name} → {tb_name}")
            else:
                self.get_logger().warn(f"No TurtleBot available for agent {agent.name} (index {i})")

        # Subscribe to hazard detection for all turtlebots (optional wildcard, depends on topic remapping)
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_callback, qos_profile_sensor_data)

        self.get_logger().info("JohannesController node initialized for multiple robots.")

    # ------------------------------------
    # Hazard detection callback
    # ------------------------------------
    def hazard_callback(self, msg):
        for hazard in msg.detections:
            if hazard.type == 1 and 'bump' in hazard.header.frame_id:
                self.get_logger().warn(f"Bumper triggered: {hazard.header.frame_id}")
                self.environment.register_bumping(None)

    # ------------------------------------
    # Core robot commands
    # ------------------------------------
    def _send_rotate_then_drive(self, agent, angle, return_after=False):
        tb = self._get_turtlebot(agent)
        if not tb:
            return

        rotate_client = tb["rotate"]
        goal = RotateAngle.Goal()
        goal.angle = angle
        goal.max_rotation_speed = 1.0

        rotate_client.wait_for_server()
        send_goal_future = rotate_client.send_goal_async(goal)
        send_goal_future.add_done_callback(lambda f: self._rotate_goal_response_callback(f, agent, angle, return_after))

    def _rotate_goal_response_callback(self, future, agent, angle, return_after):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Rotate goal was rejected.')
            self.environment.robot_reached_position()
            return

        self.get_logger().info(f'Rotate goal accepted (agent={agent.name}, angle={angle}).')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._rotate_result_callback(f, agent, angle, return_after))

    def _rotate_result_callback(self, future, agent, angle, return_after):
        result = future.result().result
        self.get_logger().info(f'Rotation complete for {agent.name} (angle={angle}).')

        if return_after and abs(angle) == 1.57:
            self._send_drive_goal(agent, 0.3, after_rotate_back=True, reverse_angle=-angle)
        else:
            self._send_drive_goal(agent, 0.3)

    def _send_drive_goal(self, agent, distance, after_rotate_back=False, reverse_angle=None):
        tb = self._get_turtlebot(agent)
        if not tb:
            return

        drive_client = tb["drive"]
        goal = DriveDistance.Goal()
        goal.distance = distance
        goal.max_translation_speed = 0.2

        drive_client.wait_for_server()
        send_goal_future = drive_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda f: self._drive_goal_response_callback(f, agent, after_rotate_back, reverse_angle)
        )

    def _drive_goal_response_callback(self, future, agent, after_rotate_back, reverse_angle):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Drive goal rejected for {agent.name}')
            return

        self.get_logger().info(f'Drive goal accepted for {agent.name}')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f: self._drive_result_callback(f, agent, after_rotate_back, reverse_angle)
        )

    def _drive_result_callback(self, future, agent, after_rotate_back, reverse_angle):
        result = future.result().result
        self.get_logger().info(f'Drive result received for {agent.name}')

        if after_rotate_back and reverse_angle is not None:
            self._send_only_rotate(agent, reverse_angle)
        else:
            self.environment.robot_reached_position()

    def _send_only_rotate(self, agent, angle):
        tb = self._get_turtlebot(agent)
        if not tb:
            return

        rotate_client = tb["rotate"]
        goal = RotateAngle.Goal()
        goal.angle = angle
        goal.max_rotation_speed = 1.0
        rotate_client.wait_for_server()
        send_goal_future = rotate_client.send_goal_async(goal)
        send_goal_future.add_done_callback(lambda f: self._only_rotate_goal_response_callback(f, agent))

    def _only_rotate_goal_response_callback(self, future, agent):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Rotate-back goal was rejected for {agent.name}.')
            self.environment.robot_reached_position()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._only_rotate_result_callback(f, agent))

    def _only_rotate_result_callback(self, future, agent):
        result = future.result().result
        self.get_logger().info(f'Rotate-back complete for {agent.name}.')
        self.environment.robot_reached_position()
    # ------------------------------------
    # Movement wrappers
    # ------------------------------------
    def move_top(self, agent):
        self.environment.robot_is_moving()
        self._send_drive_goal(agent, 0.3)

    def move_left(self, agent):
        self.environment.robot_is_moving()
        self._send_rotate_then_drive(agent, 1.57)

    def move_right(self, agent):
        self.environment.robot_is_moving()
        self._send_rotate_then_drive(agent, -1.57, return_after=True)

    def move_bottom(self, agent):
        self.environment.robot_is_moving()
        self._send_drive_goal(agent, -0.3)

    # ------------------------------------
    # Helper: map agent → turtlebot
    # ------------------------------------
    def _get_turtlebot(self, agent):
        tb = self.turtle_bots.get(agent)
        if not tb:
            self.get_logger().error(f"No TurtleBot mapping found for agent {agent.name}")
        return tb
