from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.action import DriveDistance, RotateAngle
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
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
        self.robot_should_rotate_back = False
        self.robot_rotated = False
        self.start_positions = {}
        self.turned_one_time = False
        self.turned_second_time = False
        self.current_drive_goal = None
        self.current_rotate_goal = None


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
                tb_name = self.get_turtlebot(agent)["name"]
                self.get_logger().warn(f"[{tb_name}] Bumper triggered: {hazard.header.frame_id}")
                # ✅ laufende DRIVE-ACTION abbrechen
                if self.current_drive_goal is not None:
                    self.get_logger().warn(f"[{tb_name}] Cancelling current drive goal")
                    self.current_drive_goal.cancel_goal_async()
                    self.current_drive_goal = None

                # ✅ laufende ROTATE-ACTION abbrechen
                if self.current_rotate_goal is not None:
                    self.get_logger().warn(f"[{tb_name}] Cancelling current rotate goal")
                    self.current_rotate_goal.cancel_goal_async()
                    self.current_rotate_goal = None

                # ✅ jetzt ist der Roboter "frei" für neue Kommandos
                if tb_name in self.start_positions:
                    self.return_to_start_pos(agent)
                else:
                    self.get_logger().warn(f"[{tb_name}] Keine gespeicherte Startposition – Rückfahrt nicht möglich.")

    # ------------------------------------
    # Odometry detection callback
    # ------------------------------------
    def odom_callback(self, msg, agent):
        tb_name = self.get_turtlebot(agent)["name"]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.positions[tb_name] = {"x": x, "y": y, "yaw": yaw}

    def save_start_pos(self, agent):
        tb_name = self.get_turtlebot(agent)["name"]
        timeout = time.time() + 1.0
        while tb_name not in self.positions:
            if time.time() > timeout:
                self.get_logger().warn(f"[{tb_name}] Odom-Daten nicht rechtzeitig erhalten – Startpose nicht gespeichert.")
                return
            rclpy.spin_once(self, timeout_sec=0.05)

        # Jetzt sind Odom-Daten sicher da
        self.start_positions[tb_name] = self.positions[tb_name].copy()
        self.get_logger().warn(f"[{tb_name}] Startpose gespeichert: {self.start_positions[tb_name]} (save_start_pos)")

    # ------------------------------------
    # Core robot commands
    # ------------------------------------
    def move_forward(self, agent, distance, angle):
        self.get_logger().warn(f"drive forward command received")
        goal = DriveDistance.Goal()
        goal.distance = distance
        goal.max_translation_speed = 0.2
        tb = self.get_turtlebot(agent)
        drive_client = tb["drive"]
        drive_client.wait_for_server()
        send_goal_future = drive_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda f: self.move_forward_response_callback(f, agent, distance, angle)
        )

    def move_forward_response_callback(self, future, agent, distance, angle):
        self.get_logger().warn(f"drive forward command in progress")
        goal_handle = future.result()
        self.current_drive_goal = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f: self.move_forward_result_callback(f, agent, distance, angle)
        )

    def move_forward_result_callback(self, future, agent, distance, angle):
        result = future.result()
        if result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Drive command was cancelled, ignoring result callback.")
            return
        self.get_logger().warn(f"drive forward command finished")
        if self.robot_should_rotate_back == True:
            self.get_logger().warn(f"rotate robot command send after movement")
            self.robot_should_rotate_back = False
            self.rotate_robot(agent, distance, -angle)
        else:
            self.environment.robot_reached_position()
            self.robot_rotated = False

    def rotate_robot(self, agent, distance, angle):
        self.get_logger().warn(f"rotate command received")
        goal = RotateAngle.Goal()
        self.get_logger().warn(f"angle: {angle}")
        goal.angle = angle
        goal.max_rotation_speed = 1.0
        tb = self.get_turtlebot(agent)
        rotate_client = tb["rotate"]
        rotate_client.wait_for_server()
        send_goal_future = rotate_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda f: self.rotate_robot_response_callback(f, agent, distance, angle)
        )

    def rotate_robot_response_callback(self, future, agent, distance, angle):
        self.get_logger().warn(f"rotate command in progress")
        goal_handle = future.result()
        self.current_rotate_goal = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f: self.rotate_robot_result_callback(f, agent, distance, angle)
        )
        
    def rotate_robot_result_callback(self, future, agent, distance, angle):
        result = future.result()
        if result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Rotate command was cancelled, ignoring result callback.")
            return
        self.get_logger().warn(f"rotate command finished")
        self.robot_rotated = True
        if self.robot_should_rotate_back == False:
            self.get_logger().warn(f"drive forward command send after rotation")
            self.move_forward(agent, distance, angle)
        else:
            self.environment.robot_reached_position()
            self.robot_rotated = False
            self.robot_should_rotate_back = False
    
    # ------------------------------------
    # Movements to return to start position
    # ------------------------------------

    def rotate_robot_to_start_pos(self, agent, distance, angle1, angle2):
        self.get_logger().warn(f"rotate to start pos command received")
        goal = RotateAngle.Goal()
        drive_goal = DriveDistance.Goal()
        if self.turned_one_time == False:
            goal.angle = angle1
        else:
            goal.angle = angle2
            self.turned_second_time = True
        self.get_logger().warn(f"rotate to start pos command received with angle: {goal.angle}")
        self.get_logger().warn(f"self.turned_second_time: {self.turned_second_time}")
        goal.max_rotation_speed = 1.0
        tb = self.get_turtlebot(agent)
        rotate_client = tb["rotate"]
        rotate_client.wait_for_server()
        send_goal_future = rotate_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda f: self.rotate_robot_to_start_pos_response_callback(f, agent, distance, angle1, angle2)
        )
    
    def rotate_robot_to_start_pos_response_callback(self, future, agent, distance, angle1, angle2):
        self.get_logger().warn(f"rotate to start pos command in progress")
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f: self.rotate_robot_to_start_pos_result_callback(f, agent, distance, angle1, angle2)
        )
    
    def rotate_robot_to_start_pos_result_callback(self, future, agent, distance, angle1, angle2):
        self.get_logger().warn(f"rotate to start pos command finished")
        if self.turned_second_time == False:
            self.get_logger().warn(f"drive forward to start pos command send after rotation")
            self.move_robot_to_start_pos(agent, distance, angle1, angle2)
        else:
            self.environment.robot_reached_position()

    def move_robot_to_start_pos(self, agent, distance, angle1, angle2):
        self.get_logger().warn(f"drive forward to start pos command received")
        goal = DriveDistance.Goal()
        goal.distance = distance
        goal.max_translation_speed = 0.2
        tb = self.get_turtlebot(agent)
        drive_client = tb["drive"]
        drive_client.wait_for_server()
        send_goal_future = drive_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda f: self.move_forward_to_start_pos_response_callback(f, agent, distance, angle1, angle2)
        )
        
    def move_forward_to_start_pos_response_callback(self, future, agent, distance, angle1, angle2):
        self.get_logger().warn(f"drive forward to start pos command in progress")
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f: self.move_forward_to_start_pos_result_callback(f, agent, distance, angle1, angle2)
        )

    def move_forward_to_start_pos_result_callback(self, future, agent, distance, angle1, angle2):
        self.get_logger().warn(f"drive forward to start pos command finished")
        self.get_logger().warn(f"rotate back to start orientation command send")
        self.turned_one_time = True
        self.rotate_robot_to_start_pos(agent, distance, angle1, angle2)


    
    # ------------------------------------
    # Return to start position command
    # ------------------------------------
    def return_to_start_pos(self, agent):
        tb_name = self.get_turtlebot(agent)["name"]
        if tb_name not in self.positions or tb_name not in self.start_positions:
            self.get_logger().warn(f"[{tb_name}] Keine Position oder Startposition verfügbar.(return_to_start_pos)")
            return

        current = self.positions[tb_name]
        start = self.start_positions[tb_name]

        cx, cy, cyaw = current["x"], current["y"], current["yaw"]
        sx, sy, syaw = start["x"], start["y"], start["yaw"]

        dx = sx - cx
        dy = sy - cy
        target_angle = math.atan2(dy, dx)
        rotate1 = target_angle - cyaw
        distance = math.sqrt(dx*dx + dy*dy)
        rotate2 = syaw - target_angle
        self.rotate_robot_to_start_pos(agent, distance, rotate1, rotate2)



    # ------------------------------------
    # Movement wrappers
    # ------------------------------------
    def move_top(self, agent):
        self.environment.robot_is_moving()
        self.robot_rotated = False
        self.robot_should_rotate_back = False
        self.save_start_pos(agent)
        self.get_logger().warn(f"drive forward command send")
        self.move_forward(agent, 0.3, 0.0)
        self.get_logger().warn(f"function move_top finished")

    def move_left(self, agent):
        self.environment.robot_is_moving()
        self.robot_rotated = True
        self.robot_should_rotate_back = False
        self.save_start_pos(agent)
        self.get_logger().warn(f"rotate left command send")
        self.rotate_robot(agent, 0.3, 1.57)
        self.get_logger().warn(f"function move_left finished")

    def move_right(self, agent):
        self.environment.robot_is_moving()
        self.robot_rotated = True
        self.robot_should_rotate_back = False
        self.save_start_pos(agent)
        self.get_logger().warn(f"rotate right command send")
        self.rotate_robot(agent, 0.3, -1.57)
        self.get_logger().warn(f"function move_right finished")

    def move_bottom(self, agent):
        self.environment.robot_is_moving()
        self.save_start_pos(agent)
        self.get_logger().warn(f"move backwards command send")
        self._send_drive_goal(agent, -0.3, 0.0)
        self.get_logger().warn(f"function move_bottom finished")

    # ------------------------------------
    # Helper: map agent → turtlebot
    # ------------------------------------
    def get_turtlebot(self, agent):
        tb = self.turtle_bots.get(agent)
        if not tb:
            self.get_logger().error(f"No TurtleBot mapping found for agent {agent.name}")
        return tb
