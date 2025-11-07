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
        # mappings and state per robot (keyed by tb_name)
        self.turtle_bots = {}          # {agent: {"name": tb_name, "drive": client, "rotate": client}}
        self.positions = {}            # {tb_name: {"x", "y", "yaw"}}
        self.start_positions = {}      # {tb_name: {"x", "y", "yaw"}}
        self.current_drive_goal = {}   # {tb_name: goal_handle}
        self.current_rotate_goal = {}  # {tb_name: goal_handle}
        self.return_phase = {}         # {tb_name: None | "rotate_to_start" | "drive_to_start" | "restore_orientation"}
        self.return_sequence = {}      # {tb_name: {"distance", "rotate_to_target", "final_orientation"}}


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

                # initialize per-robot state
                self.current_drive_goal[tb_name] = None
                self.current_rotate_goal[tb_name] = None
                self.return_phase[tb_name] = None
                self.return_sequence[tb_name] = {}

                self.get_logger().info(f"Mapped agent {agent.name} → {tb_name}")
            else:
                self.get_logger().warn(f"No TurtleBot available for agent {agent.name} (index {i})")

        self.get_logger().info("JohannesController node initialized for multiple robots.")

    # ------------------------------------
    # Hazard detection callback
    # ------------------------------------
    def hazard_callback(self, msg, agent):
        tb_name = self.get_turtlebot(agent)["name"]
        if self.return_phase.get(tb_name) is not None:
            return  # Robot is already returning; ignore bumps
    
        for hazard in msg.detections:
            if hazard.type == 1 and 'bump' in hazard.header.frame_id:
                self.get_logger().warn(f"[{tb_name}] Bumper triggered: {hazard.header.frame_id}")
                # Cancel current drive if exists
                if self.current_drive_goal.get(tb_name) is not None:
                    try:
                        self.get_logger().warn(f"[{tb_name}] Cancelling current drive goal")
                        self.current_drive_goal[tb_name].cancel_goal_async()
                    except Exception as e:
                        self.get_logger().error(f"[{tb_name}] Error cancelling drive goal: {e}")
                    # Keep handle until possibly overwritten by new goal

                # Cancel current rotate if exists
                if self.current_rotate_goal.get(tb_name) is not None:
                    try:
                        self.get_logger().warn(f"[{tb_name}] Cancelling current rotate goal")
                        self.current_rotate_goal[tb_name].cancel_goal_async()
                    except Exception as e:
                        self.get_logger().error(f"[{tb_name}] Error cancelling rotate goal: {e}")
                    # Keep handle until overwritten

                rclpy.spin_once(self, timeout_sec=0.05)

                # start return sequence if start pos is saved
                if tb_name in self.start_positions:
                    self.return_phase[tb_name] = "rotate_to_start"
                    self.return_to_start_pos(agent)
                else:
                    self.get_logger().warn(f"[{tb_name}] no saved start position.")

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
                self.get_logger().warn(f"[{tb_name}] No odom data - cannot store start pos.")
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
        goal.distance = float(distance)
        goal.max_translation_speed = 0.2

        drive_client = self.turtle_bots[agent]["drive"]
        drive_client.wait_for_server()

        send_goal_future = drive_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda f: self.move_forward_response_callback(f, agent, angle)
        )

    def move_forward_response_callback(self, future, agent, angle):
        tb_name = self.get_turtlebot(agent)["name"]
        self.get_logger().warn(f"drive forward command in progress")
        goal_handle = future.result()

        self.current_drive_goal[tb_name] = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f: self.move_forward_result_callback(f, agent, angle)
        )

    def move_forward_result_callback(self, future, agent, angle):
        tb_name = self.get_turtlebot(agent)["name"]
        result = future.result()
        if result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Move forward command was cancelled, ignoring result callback.")
            self.current_drive_goal[tb_name] = None
            return
        self.current_drive_goal[tb_name] = None
        self.get_logger().warn(f"drive forward command finished")

        if self.return_phase.get(tb_name) == "drive_to_start":
            self.return_phase[tb_name] = "restore_orientation"
            self.perform_final_orientation(agent)
            return
        
        if angle != 0.0:
            self.rotate_robot(agent, 0.0, -angle)
        else:
            self.environment.robot_reached_position()





    def rotate_robot(self, agent, distance, angle):
        self.get_logger().warn(f"rotate command received")
        goal = RotateAngle.Goal()
        self.get_logger().warn(f"angle: {angle}")
        goal.angle = float(angle)
        goal.max_rotation_speed = 1.0

        rotate_client = self.turtle_bots[agent]["rotate"]
        rotate_client.wait_for_server()

        send_goal_future = rotate_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda f: self.rotate_robot_response_callback(f, agent, distance, angle)
        )

    def rotate_robot_response_callback(self, future, agent, distance, angle):
        self.get_logger().warn(f"rotate command in progress")
        tb_name = self.get_turtlebot(agent)["name"]
        goal_handle = future.result()

        self.current_rotate_goal[tb_name] = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f: self.rotate_robot_result_callback(f, agent, distance, angle)
        )
        
    def rotate_robot_result_callback(self, future, agent, distance, angle):
        tb_name = self.get_turtlebot(agent)["name"]
        result = future.result()
        if result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Move forward command was cancelled, ignoring result callback.")
            self.current_rotate_goal[tb_name] = None
            return
        
        self.current_rotate_goal[tb_name] = None
        self.get_logger().warn(f"rotate command finished")

        if self.return_phase.get(tb_name) == "rotate_to_start":
            self.return_phase[tb_name] = "drive_to_start"
            self.perform_return_drive(agent)
            return
        
        if self.return_phase.get(tb_name) == "restore_orientation":
            self.get_logger().warn(f"[{tb_name}] Back at start position and orientation restored.")
            self.return_phase[tb_name] = None
            self.return_sequence[tb_name] = {}
            self.environment.robot_reached_position()
            return
        
        if distance > 0:
            self.get_logger().warn(f"drive forward command send after rotation")
            self.move_forward(agent, distance, angle)
        else:
            self.environment.register_bumping(agent)
            self.environment.robot_reached_position()

    # ------------------------------------
    # Movements to return to start position
    # ------------------------------------

    def perform_return_drive(self, agent):
        tb_name = self.get_turtlebot(agent)["name"]
        seq = self.return_sequence.get(tb_name, {})
        distance = seq.get("distance", 0.0)
        self.get_logger().warn(f"[{tb_name}] Driving back to start position... dist={distance:.3f}")
        # Important: set phase already (should already be set)
        self.move_forward(agent, distance, 0.0)

    def perform_final_orientation(self, agent):
        tb_name = self.get_turtlebot(agent)["name"]
        seq = self.return_sequence.get(tb_name, {})
        angle = seq.get("final_orientation", 0.0)
        self.get_logger().warn(f"[{tb_name}] Restoring final orientation: angle={angle:.3f}")
        self.rotate_robot(agent, 0.0, angle)

    
    # ------------------------------------
    # Return to start position command
    # ------------------------------------
    def return_to_start_pos(self, agent):
        tb_name = self.get_turtlebot(agent)["name"]

        if tb_name not in self.start_positions or tb_name not in self.positions:
            self.get_logger().error(f"[{tb_name}] Cannot return to start – missing odom or start position.")
            return

        self.get_logger().warn(f"[{tb_name}] Returning to start position...")

        # Compute direction & distance
        start = self.start_positions[tb_name]
        current = self.positions[tb_name]

        dx = start["x"] - current["x"]
        dy = start["y"] - current["y"]

        # Distance
        distance = math.sqrt(dx*dx + dy*dy)

        # Angle to face the start point
        target_angle = math.atan2(dy, dx)
        rotate_to_target = target_angle - current["yaw"]

        # Normalize angle
        rotate_to_target = math.atan2(math.sin(rotate_to_target), math.cos(rotate_to_target))

        # Angle to restore original orientation
        final_orientation = start["yaw"] - target_angle
        final_orientation = math.atan2(math.sin(final_orientation), math.cos(final_orientation))

        # Store state for sequenced return
        self.return_sequence[tb_name] = {
            "distance": float(distance),
            "rotate_to_target": float(rotate_to_target),
            "final_orientation": float(final_orientation)
        }

        self.return_phase[tb_name] = "rotate_to_start"

        self.get_logger().warn(
            f"[{tb_name}] Return movement:"
            f" rotate_to_target={rotate_to_target:.2f},"
            f" drive={distance:.2f}m,"
            f" final_yaw={final_orientation:.2f}"
        )

        # Start sequence: rotate towards start
        self.rotate_robot(agent, 0.0, rotate_to_target)



    # ------------------------------------
    # Movement wrappers
    # ------------------------------------
    def move_top(self, agent):
        self.environment.robot_is_moving()
        self.save_start_pos(agent)
        self.get_logger().warn(f"drive forward command send")
        self.move_forward(agent, 0.3, 0.0)
        self.get_logger().warn(f"function move_top finished")

    def move_left(self, agent):
        self.environment.robot_is_moving()
        self.save_start_pos(agent)
        self.get_logger().warn(f"rotate left command send")
        self.rotate_robot(agent, 0.3, 1.57)
        self.get_logger().warn(f"function move_left finished")

    def move_right(self, agent):
        self.environment.robot_is_moving()
        self.save_start_pos(agent)
        self.get_logger().warn(f"rotate right command send")
        self.rotate_robot(agent, 0.3, -1.57)
        self.get_logger().warn(f"function move_right finished")

    def move_bottom(self, agent):
        self.environment.robot_is_moving()
        self.save_start_pos(agent)
        self.get_logger().warn(f"move backwards command send")
        self.rotate_robot(agent, 0.3, 3.14)
        self.get_logger().warn(f"function move_bottom finished")

    # ------------------------------------
    # Helper: map agent → turtlebot
    # ------------------------------------
    def get_turtlebot(self, agent):
        tb = self.turtle_bots.get(agent)
        if not tb:
            self.get_logger().error(f"No TurtleBot mapping found for agent {agent.name}")
        return tb
