import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

from .swarm_logic import update_swarm, load_json_params

class SwarmController(Node):
    def __init__(self):
        super().__init__("swarm_controller")
        default_uavs = ["uav_1", "uav_2", "uav_3", "uav_4", "uav_5"]
        self.declare_parameter("uav_names", default_uavs)
        self.uav_names = self.get_parameter("uav_names").value

        self.odom = {name: None for name in self.uav_names}
        self.cmd_pubs = {}
        self.odom_subs = []

        for name in self.uav_names:
            self.cmd_pubs[name] = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
            sub = self.create_subscription(
                Odometry, f"/{name}/odom",
                lambda msg, n=name: self.odom_callback(msg, n), 10)
            self.odom_subs.append(sub)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.dt = 0.1

        N = len(self.uav_names)
        self.positions = np.zeros((N, 2))
        self.velocities = np.zeros((N, 2))
        self.leader_pos = np.array([0.0, 0.0])
        self.leader_vel = np.array([0.0, 0.0])

        # Optional: altitude hold
        self.altitudes = {name: 0.0 for name in self.uav_names}
        self.target_altitude = 5.0
        self.altitude_gain = 0.8


    def odom_callback(self, msg: Odometry, uav_name: str):
        idx = self.uav_names.index(uav_name)
        self.positions[idx, 0] = msg.pose.pose.position.x
        self.positions[idx, 1] = msg.pose.pose.position.y
        self.altitudes[uav_name] = msg.pose.pose.position.z

    def control_loop(self):
        # 1) Read params (for WAYPOINTS and swarm weights)
        params = load_json_params()
        waypoints = params.get("WAYPOINTS", [])

        # 2) Move leader toward first waypoint if any
        if waypoints:
            target = np.array(waypoints[0], dtype=float)
            direction = target - self.leader_pos
            dist = np.linalg.norm(direction)
            if dist > 1e-3:
                direction /= dist
                speed = 0.5  # m/s leader speed
                self.leader_vel = direction * speed
                self.leader_pos = self.leader_pos + self.leader_vel * self.dt

            self.get_logger().info(
                f"Leader_pos: {self.leader_pos}, target: {target}, dist: {dist} "
            )
            # (if very close, leader just stays near the target)

        # 3) Update swarm positions and velocities
        self.positions, self.velocities = update_swarm(
            self.positions, self.velocities,
            self.leader_pos, self.leader_vel
        )

        # 4) Publish velocities to all drones (with altitude hold)
        for i, name in enumerate(self.uav_names):
            cmd = Twist()
            cmd.linear.x = self.velocities[i, 0]
            cmd.linear.y = self.velocities[i, 1]

            # altitude hold around 5 m
            z = self.altitudes[name]
            error_z = self.target_altitude - z
            vz = self.altitude_gain * error_z
            vz = max(min(vz, 1.0), -1.0)
            cmd.linear.z = vz

            cmd.angular.z = 0.0
            self.cmd_pubs[name].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down swarm_controller")
    finally:
        node.destroy_node()
        rclpy.shutdown()