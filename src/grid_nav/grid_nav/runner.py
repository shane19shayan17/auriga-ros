import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus

from grid_nav.map_utils import (parse_text_map, cell_to_world, path_to_cells,
                                render)


class Runner(Node):
    """Asks Nav2's planner_server for a path, then 'walks' the point along it
    and prints the map in the terminal after every step."""

    def __init__(self):
        super().__init__('grid_runner')
        self.declare_parameter('map_file', '')
        self.declare_parameter('resolution', 1.0)
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('step_delay', 0.3)
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def _pose(self, cell, res):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = cell_to_world(cell, res)
        p.pose.orientation.w = 1.0
        return p

    def plan(self, start, goal, res):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'action server /compute_path_to_pose not available '
                '(is planner_server ACTIVE? check: ros2 lifecycle get /planner_server)')
            return None
        g = ComputePathToPose.Goal()
        g.start = self._pose(start, res)
        g.goal = self._pose(goal, res)
        g.planner_id = self.get_parameter('planner_id').value
        g.use_start = True   # else Nav2 takes the start from TF, not from us

        fut = self.client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            return None
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        wrapper = res_fut.result()
        result = wrapper.result
        # Humble has no error_code; newer distros do. getattr works on both.
        code = getattr(result, 'error_code', 0)
        if (wrapper.status != GoalStatus.STATUS_SUCCEEDED
                or code != 0 or not result.path.poses):
            msg = getattr(result, 'error_msg', '')
            self.get_logger().error(
                f'planning failed: status={wrapper.status} error_code={code} {msg} '
                '(see the planner_server log in the launch terminal for the reason)')
            return None
        self.get_logger().info(
            f'path with {len(result.path.poses)} poses')
        return [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]


def main():
    rclpy.init()
    node = Runner()
    try:
        path_file = node.get_parameter('map_file').value
        res = float(node.get_parameter('resolution').value)
        delay = float(node.get_parameter('step_delay').value)
        with open(path_file, 'r', encoding='utf-8') as f:
            gm = parse_text_map(f.read(), resolution=res)
        if gm.start is None or gm.goal is None:
            node.get_logger().error("map needs both 'R' (robot) and 'G' (goal)")
            return

        pts = node.plan(gm.start, gm.goal, res)
        if pts is None:
            return
        cells = path_to_cells(pts, res)
        bad = [c for c in cells if c in gm.walls]
        if bad:
            node.get_logger().warn(f'path crosses wall cells {bad}: smoothing cut a corner?')

        for c in cells:
            sys.stdout.write('\033[H\033[J')          # clear terminal
            print(render(gm, path=cells, robot=c))
            print(f'\nrobot at {c}  goal {gm.goal}  steps {len(cells) - 1}')
            sys.stdout.flush()
            time.sleep(delay)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
