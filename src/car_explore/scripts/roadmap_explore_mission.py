#!/usr/bin/env python3
"""启动 Roadmap Explorer 新建图任务，并在探索完成后返回启动位姿。"""

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from roadmap_explorer_msgs.action import Explore
from tf2_ros import Buffer, TransformException, TransformListener


class RoadmapExploreMission(Node):
    def __init__(self):
        super().__init__('roadmap_explore_mission')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('return_to_start', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.return_to_start = bool(self.get_parameter('return_to_start').value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.explore_client = ActionClient(self, Explore, 'roadmap_explorer')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.start_pose = None
        self.started = False
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self._try_start)

    def _try_start(self):
        if self.started:
            return
        if (self.get_clock().now() - self.start_time).nanoseconds < self.start_delay * 1e9:
            return
        if not self.explore_client.server_is_ready():
            self.get_logger().info('Waiting for roadmap_explorer action server...')
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.5))
        except TransformException as exc:
            self.get_logger().warning(f'Waiting for {self.map_frame}->{self.base_frame}: {exc}')
            return

        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = self.map_frame
        self.start_pose.header.stamp = self.get_clock().now().to_msg()
        self.start_pose.pose.position.x = transform.transform.translation.x
        self.start_pose.pose.position.y = transform.transform.translation.y
        self.start_pose.pose.position.z = transform.transform.translation.z
        self.start_pose.pose.orientation = transform.transform.rotation

        self.started = True
        self.timer.cancel()
        self.get_logger().info(
            'Recorded start pose '
            f'({self.start_pose.pose.position.x:.2f}, {self.start_pose.pose.position.y:.2f}); '
            'starting Roadmap Explorer')

        goal = Explore.Goal()
        goal.exploration_bringup_mode = Explore.Goal.NEW_EXPLORATION_SESSION
        goal.load_from_folder.data = ''
        goal.session_name.data = 'smartcar_mapping'
        future = self.explore_client.send_goal_async(goal)
        future.add_done_callback(self._explore_goal_response)

    def _explore_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Roadmap Explorer rejected the exploration goal')
            return
        self.get_logger().info('Roadmap Explorer accepted the exploration goal')
        goal_handle.get_result_async().add_done_callback(self._explore_result)

    def _explore_result(self, future):
        wrapped = future.result()
        result = wrapped.result
        self.get_logger().info(
            f'Exploration finished: status={wrapped.status}, '
            f'success={result.success}, error_code={result.error_code}')

        if not result.success:
            self.get_logger().error('Exploration failed; not returning automatically')
            return
        if not self.return_to_start:
            self.get_logger().info('Return-to-start disabled; mission complete')
            return
        self._send_return_goal()

    def _send_return_goal(self):
        goal = NavigateToPose.Goal()
        self.start_pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = self.start_pose
        self.get_logger().info('Exploration complete; returning to recorded start pose')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._return_goal_response)

    def _return_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the return-to-start goal')
            return
        goal_handle.get_result_async().add_done_callback(self._return_result)

    def _return_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Returned to the recorded start pose successfully')
        else:
            self.get_logger().error(f'Return-to-start failed with action status {status}')


def main(args=None):
    rclpy.init(args=args)
    node = RoadmapExploreMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
