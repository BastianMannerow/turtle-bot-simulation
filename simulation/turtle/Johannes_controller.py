from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.action import DriveDistance, RotateAngle
from nav_msgs.msg import Odometry
import math
import rclpy
import time

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
        self.positions = {}

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
                # Subscribe to hazard detection for every single turtlebot
                self.create_subscription(HazardDetectionVector, f"/{tb_name}/hazard_detection",lambda msg, a=agent: self.hazard_callback(msg, a), qos_profile_sensor_data)
                # Subscribe to odometry topic for every single turtlebot
                self.create_subscription(Odometry, f"/{tb_name}/odom", lambda msg, a=agent: self.odom_callback(msg, a), qos_profile_sensor_data)

                self.positions[tb_name] = {"x": 0.0, "y": 0.0, "yaw": 0.0}

                self.get_logger().info(f"Mapped agent {agent.name} → {tb_name}")
            else:
                self.get_logger().warn(f"No TurtleBot available for agent {agent.name} (index {i})")

        self.get_logger().info("JohannesController node initialized for multiple robots.")

    # ------------------------------------
    # Hazard detection callback
    # ------------------------------------
    def hazard_callback(self, msg, agent):
        for hazard in msg.detections:
            if hazard.type == 1 and 'bump' in hazard.header.frame_id:
                tb_name = self._get_turtlebot(agent)["name"]
                self.get_logger().warn(f"[{tb_name}] Bumper triggered: {hazard.header.frame_id}")
                self.bump_triggered = True
                self.return_to_start_pos(agent)
            else:
                return
            
    def return_to_start_pos(self, agent):
        tb_name = self._get_turtlebot(agent)["name"]
        current_pos = self.positions.get(tb_name, None)
        self.get_logger().warn(f"self.start_pos: {self.start_pos}")
        self.get_logger().warn(f"current_pos: {current_pos}")
        dx = self.start_pos["x"] - current_pos["x"]
        dy = self.start_pos["y"] - current_pos["y"]
        distance = (dx**2 + dy**2) ** 0.5
        self.get_logger().warn(f"distance to back up: {distance}")
        target_angle = math.atan2(dy, dx)
        self.get_logger().warn(f"target_angle: {target_angle}")
        yaw = current_pos["yaw"]
        self.get_logger().warn(f"yaw: {yaw}")
        rotate_angle = target_angle - yaw
        rotate_angle = math.atan2(math.sin(rotate_angle), math.cos(rotate_angle))
        self.get_logger().warn(f"rotate_angle: {rotate_angle}")

        if self.bump_triggered:
            self._send_only_rotate(agent, rotate_angle)
            time.sleep(5)
            self._send_drive_goal(agent, distance)
            time.sleep(3)
            self._send_only_rotate(agent, -rotate_angle)
            time.sleep(5)
            self.environment.register_bumping(agent)
            self.bump_triggered = False

    # ------------------------------------
    # Odometry detection callback
    # ------------------------------------
    def odom_callback(self, msg, agent):
        tb_name = self._get_turtlebot(agent)["name"]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.positions[tb_name] = {"x": x, "y": y, "yaw": yaw}

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
        tb_name = self._get_turtlebot(agent)["name"]
        self.start_pos = self.positions.get(tb_name, None)
        self._send_drive_goal(agent, 0.3)

    def move_left(self, agent):
        self.environment.robot_is_moving()
        tb_name = self._get_turtlebot(agent)["name"]
        self.start_pos = self.positions.get(tb_name, None)
        self._send_rotate_then_drive(agent, 1.57, return_after=True)

    def move_right(self, agent):
        self.environment.robot_is_moving()
        tb_name = self._get_turtlebot(agent)["name"]
        self.start_pos = self.positions.get(tb_name, None)
        self._send_rotate_then_drive(agent, -1.57, return_after=True)

    def move_bottom(self, agent):
        self.environment.robot_is_moving()
        tb_name = self._get_turtlebot(agent)["name"]
        self.start_pos = self.positions.get(tb_name, None)
        self._send_drive_goal(agent, -0.3)

    # ------------------------------------
    # Helper: map agent → turtlebot
    # ------------------------------------
    def _get_turtlebot(self, agent):
        tb = self.turtle_bots.get(agent)
        if not tb:
            self.get_logger().error(f"No TurtleBot mapping found for agent {agent.name}")
        return tb
