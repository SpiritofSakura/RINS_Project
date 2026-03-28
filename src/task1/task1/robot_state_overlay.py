#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from rviz_2d_overlay_msgs.msg import OverlayText


DOVOLJENA_STANJA = {
    'IDLE',
    'MANUAL_CONTROL',
    'PATROL',
    'APPROACH_FACE',
    'INTERACT_FACE',
    'APPROACH_RING',
    'INTERACT_RING',
    'RETURN_TO_PATROL',
}


class RobotStateOverlay(Node):
    def __init__(self):
        super().__init__('robot_state_overlay')

        self.trenutno_stanje = 'IDLE'

        self.sub_stanje = self.create_subscription(
            String,
            '/robot_state',
            self.obdelaj_stanje,
            10
        )

        self.pub_overlay = self.create_publisher(
            OverlayText,
            '/robot_state_overlay',
            10
        )

        self.casovnik = self.create_timer(0.1, self.objavi_overlay)

        self.get_logger().info('Robot state overlay started.')

    def obdelaj_stanje(self, msg: String):
        novo_stanje = msg.data.strip()

        if novo_stanje not in DOVOLJENA_STANJA:
            self.get_logger().warn(f'Unknown robot state received: {novo_stanje}')
            return

        if novo_stanje != self.trenutno_stanje:
            self.trenutno_stanje = novo_stanje
            self.get_logger().info(f'Robot state changed to: {self.trenutno_stanje}')

    def barva_ozadja(self) -> ColorRGBA:
        bar = ColorRGBA()
        bar.r = 0.0
        bar.g = 0.0
        bar.b = 0.0
        bar.a = 0.55
        return bar

    def barva_besedila(self) -> ColorRGBA:
        bar = ColorRGBA()

        if self.trenutno_stanje == 'IDLE':
            bar.r = 0.8
            bar.g = 0.8
            bar.b = 0.8
            bar.a = 1.0
            return bar

        if self.trenutno_stanje == 'MANUAL_CONTROL':
            bar.r = 1.0
            bar.g = 0.6
            bar.b = 0.2
            bar.a = 1.0
            return bar

        if self.trenutno_stanje == 'PATROL':
            bar.r = 1.0
            bar.g = 1.0
            bar.b = 1.0
            bar.a = 1.0
            return bar

        if self.trenutno_stanje in {'APPROACH_FACE', 'INTERACT_FACE'}:
            bar.r = 0.4
            bar.g = 1.0
            bar.b = 0.4
            bar.a = 1.0
            return bar

        if self.trenutno_stanje in {'APPROACH_RING', 'INTERACT_RING'}:
            bar.r = 1.0
            bar.g = 0.9
            bar.b = 0.2
            bar.a = 1.0
            return bar

        if self.trenutno_stanje == 'RETURN_TO_PATROL':
            bar.r = 0.4
            bar.g = 0.8
            bar.b = 1.0
            bar.a = 1.0
            return bar

        bar.r = 1.0
        bar.g = 1.0
        bar.b = 1.0
        bar.a = 1.0
        return bar

    def objavi_overlay(self):
        msg = OverlayText()

        msg.action = OverlayText.ADD

        msg.width = 320
        msg.height = 70

        msg.horizontal_alignment = OverlayText.LEFT
        msg.vertical_alignment = OverlayText.TOP
        msg.horizontal_distance = 15
        msg.vertical_distance = 15

        msg.bg_color = self.barva_ozadja()
        msg.fg_color = self.barva_besedila()

        msg.line_width = 2
        msg.text_size = 18.0
        msg.font = 'DejaVu Sans Mono'

        msg.text = f'ROBOT STATE\n{self.trenutno_stanje}'

        self.pub_overlay.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    vozlisce = RobotStateOverlay()
    rclpy.spin(vozlisce)
    vozlisce.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()