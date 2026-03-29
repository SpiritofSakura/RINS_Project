#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String


class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        self.manualno_aktivno = False
        self.patrola_zahtevana = False
        self.patrola_koncana = False
        self.trenutno_stanje = 'IDLE'

        qos_lat = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.pub_sta = self.create_publisher(String, '/robot_state', qos_lat)
        self.pub_pat = self.create_publisher(Bool, '/patrol_enabled', qos_lat)

        self.sub_roc = self.create_subscription(
            Bool,
            '/manual_control_active',
            self.manual_callback,
            qos_lat
        )

        self.sub_ukp = self.create_subscription(
            Bool,
            '/patrol_command',
            self.patrol_command_callback,
            qos_lat
        )

        self.sub_kon = self.create_subscription(
            Bool,
            '/patrol_finished',
            self.patrol_finished_callback,
            qos_lat
        )

        self.osvezi_stanje()

        self.get_logger().info('Behavior manager started.')

    def objavi_stanje(self, stanje):
        if stanje == self.trenutno_stanje:
            return

        self.trenutno_stanje = stanje
        msg = String()
        msg.data = stanje
        self.pub_sta.publish(msg)
        self.get_logger().info(f'Robot state -> {stanje}')

    def objavi_patrol(self, omogoceno):
        msg = Bool()
        msg.data = omogoceno
        self.pub_pat.publish(msg)

    def osvezi_stanje(self):
        if self.manualno_aktivno:
            self.objavi_patrol(False)
            self.objavi_stanje('MANUAL_CONTROL')
            return

        if self.patrola_zahtevana and not self.patrola_koncana:
            self.objavi_patrol(True)
            self.objavi_stanje('PATROL')
            return

        self.objavi_patrol(False)
        self.objavi_stanje('IDLE')

    def manual_callback(self, msg: Bool):
        self.manualno_aktivno = msg.data
        self.osvezi_stanje()

    def patrol_command_callback(self, msg: Bool):
        self.patrola_zahtevana = msg.data

        if self.patrola_zahtevana:
            self.patrola_koncana = False

        self.osvezi_stanje()

    def patrol_finished_callback(self, msg: Bool):
        if not msg.data:
            return

        self.patrola_koncana = True
        self.patrola_zahtevana = False
        self.osvezi_stanje()


def main(args=None):
    rclpy.init(args=args)
    vozlisce = BehaviorManager()
    rclpy.spin(vozlisce)
    vozlisce.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()