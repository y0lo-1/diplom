#!/usr/bin/env python3
"""
Узел-клиент для отправки целевых точек в стек навигации Nav2.
Использует action /navigate_to_pose.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        self.get_logger().info('Ожидание сервера навигации...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Сервер /navigate_to_pose недоступен.')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        from math import sin, cos
        goal.pose.pose.orientation.z = sin(yaw / 2.0)
        goal.pose.pose.orientation.w = cos(yaw / 2.0)

        self.get_logger().info(f'Отправляю цель: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        future = self._client.send_goal_async(goal, feedback_callback=self.fb_cb)
        future.add_done_callback(self.goal_response_cb)

    def fb_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Осталось {fb.distance_remaining:.2f} м',
            throttle_duration_sec=2.0)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Цель отклонена.')
            return
        self.get_logger().info('Цель принята, движемся.')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        self.get_logger().info(f'Финальный статус: {status}')
        rclpy.shutdown()


def main():
    import sys
    rclpy.init()
    node = GoalSender()
    if len(sys.argv) >= 3:
        x, y = float(sys.argv[1]), float(sys.argv[2])
        yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    else:
        x, y, yaw = 2.0, 1.0, 0.0
    node.send_goal(x, y, yaw)
    rclpy.spin(node)


if __name__ == '__main__':
    main()