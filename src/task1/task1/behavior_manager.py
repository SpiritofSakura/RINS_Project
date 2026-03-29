#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String, Empty
from visualization_msgs.msg import Marker


class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        self.manualno_aktivno = False
        self.patrola_zahtevana = False
        self.patrola_koncana = False
        self.trenutno_stanje = 'IDLE'

        self.aktivna_tarca = None
        self.obdelane_tarke = []

        self.prag_ujemanja = 0.6

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

        self.sub_tar_done = self.create_subscription(
            Empty,
            '/target_done',
            self.target_done_callback,
            10
        )

        self.sub_res_pat = self.create_subscription(
            Empty,
            '/resume_patrol',
            self.resume_patrol_callback,
            10
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

        self.sub_obr = self.create_subscription(
            Marker,
            '/detected_face_locations',
            self.face_callback,
            10
        )

        self.sub_prs = self.create_subscription(
            Marker,
            '/detected_ring_locations',
            self.ring_callback,
            10
        )

        self.osvezi_stanje()

        self.get_logger().info('Behavior manager started.')

    def razdalja(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

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

        if self.aktivna_tarca is not None:
            if self.aktivna_tarca['tip'] == 'face':
                self.objavi_patrol(False)
                self.objavi_stanje('APPROACH_FACE')
                return

            if self.aktivna_tarca['tip'] == 'ring':
                self.objavi_patrol(False)
                self.objavi_stanje('APPROACH_RING')
                return

        if self.patrola_zahtevana and not self.patrola_koncana:
            self.objavi_patrol(True)
            self.objavi_stanje('PATROL')
            return

        self.objavi_patrol(False)
        self.objavi_stanje('IDLE')

    def je_ze_obdelana(self, tip, x, y):
        for tar in self.obdelane_tarke:
            if tar['tip'] != tip:
                continue

            raz = self.razdalja(x, y, tar['x'], tar['y'])
            if raz <= self.prag_ujemanja:
                return True

        return False
    

    def target_done_callback(self, msg: Empty):
        if self.aktivna_tarca is None:
            self.get_logger().info('No active target to mark as done.')
            return

        self.obdelane_tarke.append(self.aktivna_tarca)

        self.get_logger().info(
            f"Target completed -> type={self.aktivna_tarca['tip']}, "
            f"x={self.aktivna_tarca['x']:.2f}, y={self.aktivna_tarca['y']:.2f}"
        )

        self.aktivna_tarca = None
        self.osvezi_stanje()

    def resume_patrol_callback(self, msg: Empty):
        if self.aktivna_tarca is not None:
            self.get_logger().info('Clearing active target and resuming patrol.')
            self.aktivna_tarca = None
        else:
            self.get_logger().info('No active target. Refreshing state.')

        self.osvezi_stanje()

    def nastavi_aktivno_tarco(self, tip, x, y, z):
        if self.aktivna_tarca is not None:
            return

        self.aktivna_tarca = {
            'tip': tip,
            'x': x,
            'y': y,
            'z': z,
        }

        self.get_logger().info(
            f'New target selected -> type={tip}, x={x:.2f}, y={y:.2f}, z={z:.2f}'
        )

        self.osvezi_stanje()

    def face_callback(self, msg: Marker):
        if self.trenutno_stanje != 'PATROL':
            return

        if self.aktivna_tarca is not None:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        if self.je_ze_obdelana('face', x, y):
            return

        self.nastavi_aktivno_tarco('face', x, y, z)

    def ring_callback(self, msg: Marker):
        if self.trenutno_stanje != 'PATROL':
            return

        if self.aktivna_tarca is not None:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        if self.je_ze_obdelana('ring', x, y):
            return

        self.nastavi_aktivno_tarco('ring', x, y, z)

    def manual_callback(self, msg: Bool):
        self.manualno_aktivno = msg.data
        self.osvezi_stanje()

    def patrol_command_callback(self, msg: Bool):
        self.patrola_zahtevana = msg.data

        if self.patrola_zahtevana:
            self.patrola_koncana = False

        if not self.patrola_zahtevana and self.aktivna_tarca is None and not self.manualno_aktivno:
            self.objavi_patrol(False)

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