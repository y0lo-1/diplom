#!/usr/bin/env python3
"""
Узел телеуправления роботом TurtleBro с клавиатуры.
Подписывается на /scan для аварийной остановки при препятствиях.
Публикует команды в /cmd_vel.
"""
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

INSTRUCTIONS = """
====================================================
   TurtleBro - телеуправление с клавиатуры
====================================================
   Движение:        Скорости:
        w           w/x : +/- линейная скорость
   a    s    d      a/d : +/- угловая скорость
        x           s   : полная остановка
                    space : экстренный СТОП
   q : выход

   Лимиты: v_max = 0.22 м/с, w_max = 2.0 рад/с
====================================================
"""

MOVE_BINDINGS = {
    'w': (1, 0), 'x': (-1, 0),
    'a': (0, 1), 'd': (0, -1),
    's': (0, 0),
}


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        self.declare_parameter('linear_step', 0.02)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('safety_distance', 0.25)
        self.declare_parameter('enable_safety', True)

        self.lin_step = self.get_parameter('linear_step').value
        self.ang_step = self.get_parameter('angular_step').value
        self.lin_max = self.get_parameter('max_linear').value
        self.ang_max = self.get_parameter('max_angular').value
        self.safe_dist = self.get_parameter('safety_distance').value
        self.safety_on = self.get_parameter('enable_safety').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_cb, 10)

        self.target_lin = 0.0
        self.target_ang = 0.0
        self.front_distance = float('inf')

        self.timer = self.create_timer(0.1, self.publish_cmd)
        self.get_logger().info('Teleop запущен. См. инструкцию в терминале.')

    def scan_cb(self, msg: LaserScan):
        """Извлекаем минимальное расстояние в секторе +/- 15 градусов впереди."""
        n = len(msg.ranges)
        if n == 0:
            return
        sector = 15  # градусов
        idx_per_deg = n / 360.0
        half = int(sector * idx_per_deg)
        front_idxs = list(range(0, half)) + list(range(n - half, n))
        valid = [msg.ranges[i] for i in front_idxs
                 if msg.range_min < msg.ranges[i] < msg.range_max]
        self.front_distance = min(valid) if valid else float('inf')

    def publish_cmd(self):
        twist = Twist()
        lin = self.target_lin
        ang = self.target_ang

        # Защита: блокировка движения вперёд при препятствии
        if self.safety_on and lin > 0 and self.front_distance < self.safe_dist:
            lin = 0.0
            self.get_logger().warn(
                f'Препятствие в {self.front_distance:.2f} м - стоп!',
                throttle_duration_sec=1.0)

        twist.linear.x = lin
        twist.angular.z = ang
        self.pub.publish(twist)

    def update_velocity(self, key):
        if key == ' ':
            self.target_lin = 0.0
            self.target_ang = 0.0
            return
        if key in MOVE_BINDINGS:
            dlin, dang = MOVE_BINDINGS[key]
            if key == 's':
                self.target_lin = 0.0
                self.target_ang = 0.0
            else:
                self.target_lin = max(-self.lin_max,
                                      min(self.lin_max,
                                          self.target_lin + dlin * self.lin_step))
                self.target_ang = max(-self.ang_max,
                                      min(self.ang_max,
                                          self.target_ang + dang * self.ang_step))
        self.print_status()

    def print_status(self):
        print(f'\rv = {self.target_lin:+.2f} м/с | '
              f'w = {self.target_ang:+.2f} рад/с | '
              f'фронт: {self.front_distance:.2f} м    ', end='')


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopKeyboard()
    print(INSTRUCTIONS)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key(settings)
            if key == 'q':
                break
            if key:
                node.update_velocity(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # стоп
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()