#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
import math

# Update these to match your actual maze coordinates (from RViz2 hover)
WAYPOINTS = [
    (4.66,-0.0508, 0.00247),
    (4.78, 2.16, 0.0103),
    (-2.17, 2.02, 0.00247),
    (-6.22, 1.23, -0.00143),
    (-6.31, -1.83, 0.00247),
    (-1.32, -1.72, -0.00534),
    (-0.493, -4.47, -0.00534),
    (0.86, -4.45, -0.00534),
    (0.751, -1.14, -0.00534),
    (-1.98, -0.272, -0.00143),
]

PAUSE_DURATION_SEC = 5.0
AVOIDANCE_RADIUS = 0.4  # meters — how wide a berth to give the trash spot


def yaw_to_quaternion(yaw_deg):
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._waypoints = WAYPOINTS
        self._index = 0
        self._goal_handle = None
        self._paused = False
        self._avoid_zones = []  # list of (x, y) to steer clear of

        # Listen for trash alerts from the detector
        self.alert_sub = self.create_subscription(
            PointStamped, '/trash_alert', self.alert_callback, 10
        )

        self.get_logger().info('Patrol node started, waiting for Nav2...')
        self._client.wait_for_server()
        self.send_next_goal()

    # ---------- Normal patrol logic ----------

    def get_safe_waypoint(self, x, y, yaw_deg):
        """Nudge a waypoint away from any known trash spot if too close."""
        for (ax, ay) in self._avoid_zones:
            dist = math.hypot(x - ax, y - ay)
            if dist < AVOIDANCE_RADIUS:
                self.get_logger().warn(
                    f'Waypoint ({x:.2f},{y:.2f}) too close to trash at ({ax:.2f},{ay:.2f}), nudging path'
                )
                # Push the waypoint away from the trash spot along the line between them
                if dist > 0.01:
                    dx, dy = (x - ax) / dist, (y - ay) / dist
                else:
                    dx, dy = 1.0, 0.0
                x = ax + dx * AVOIDANCE_RADIUS
                y = ay + dy * AVOIDANCE_RADIUS
        return x, y

    def send_next_goal(self):
        if self._paused:
            return  # don't send a new goal while paused

        x, y, yaw_deg = self._waypoints[self._index]
        x, y = self.get_safe_waypoint(x, y, yaw_deg)
        _, _, qz, qw = yaw_to_quaternion(yaw_deg)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f'Going to waypoint {self._index}: ({x:.2f}, {y:.2f})')
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self.goal_accepted_callback)

    def goal_accepted_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected, retrying...')
            self.send_next_goal()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future):
        if self._paused:
            return  # result arrived after we already cancelled for a pause
        self.get_logger().info(f'Reached waypoint {self._index}')
        self._index = (self._index + 1) % len(self._waypoints)
        self.send_next_goal()

    # ---------- Trash alert handling ----------

    def alert_callback(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y

        # Remember this spot so future waypoints route around it (deduplicate)
        if not any(math.hypot(x - az[0], y - az[1]) < AVOIDANCE_RADIUS for az in self._avoid_zones):
            self._avoid_zones.append((x, y))

        if self._paused:
            return  # already handling a stop, ignore duplicate alerts

        self.get_logger().info(
            f'Trash alert received at ({x:.2f},{y:.2f}) — pausing patrol for {PAUSE_DURATION_SEC}s'
        )
        self._paused = True

        # Cancel the current Nav2 goal so the robot actually stops
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        # Resume after the pause, using a one-shot timer (non-blocking)
        self._pause_timer = self.create_timer(PAUSE_DURATION_SEC, self.resume_after_pause)

    def resume_after_pause(self):
        self._pause_timer.cancel()
        self._paused = False
        self.get_logger().info('Pause complete — resuming patrol with avoidance')
        self.send_next_goal()


def main():
    rclpy.init()
    node = PatrolNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
