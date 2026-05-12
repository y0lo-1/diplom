#!/usr/bin/env python3
"""
Простой реактивный алгоритм обхода препятствий.
Анализирует показания лидара в трёх секторах: левом, центральном, правом.
Реализует поведение: двигаться вперёд / повернуть в более свободный сектор.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('critical_distance', 0.25)

        self.v = self.get_parameter('linear_speed').value
        self.w = self.get_parameter('angular_speed').value
        self.d_safe = self.get_parameter('safe_distance').value
        self.d_crit = self.get_parameter('critical_distance').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)

        self.state = 'FORWARD'
        self.get_logger().info('Узел обхода препятствий запущен.')

    def get_sector_min(self, ranges, n, deg_from, deg_to):
        """Минимальная дистанция в угловом секторе в градусах от 0 (вперёд)."""
        idx_per_deg = n / 360.0
        i_from = int(deg_from * idx_per_deg) % n
        i_to = int(deg_to * idx_per_deg) % n
        if i_from < i_to:
            sector = ranges[i_from:i_to]
        else:
            sector = list(ranges[i_from:]) + list(ranges[:i_to])
        valid = [r for r in sector if 0.05 < r < 10.0 and not math.isinf(r)]
        return min(valid) if valid else float('inf')

    def scan_cb(self, msg: LaserScan):
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        # Сектора: впереди +/- 20, слева 30..90, справа 270..330
        front = self.get_sector_min(ranges, n, 340, 20)
        left  = self.get_sector_min(ranges, n, 30, 90)
        right = self.get_sector_min(ranges, n, 270, 330)

        cmd = Twist()

        if front < self.d_crit:
            # критическое препятствие — отъехать назад
            self.state = 'BACKUP'
            cmd.linear.x = -self.v * 0.5
            cmd.angular.z = self.w if left > right else -self.w
        elif front < self.d_safe:
            # есть препятствие — поворачиваем в более свободную сторону
            self.state = 'TURN'
            cmd.linear.x = 0.0
            cmd.angular.z = self.w if left > right else -self.w
        else:
            self.state = 'FORWARD'
            cmd.linear.x = self.v
            cmd.angular.z = 0.0

        self.pub.publish(cmd)

        self.get_logger().info(
            f'[{self.state}] F={front:.2f} L={left:.2f} R={right:.2f}',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # стоп
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()