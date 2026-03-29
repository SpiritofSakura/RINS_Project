#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String, Empty
from visualization_msgs.msg import Marker


def yaw_v_kvaternion(kot):
    kvaternion = Quaternion()
    kvaternion.z = math.sin(kot / 2.0)
    kvaternion.w = math.cos(kot / 2.0)
    return kvaternion


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

        self.zadnja_poza_robota = None
        self.shranjena_poza = None

        self.akcijski_odjemalec = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_streznik_pripravljen = False
        self.cakanje_na_sprejem = False
        self.cilj_aktiven = False
        self.rocaj_cilja = None
        self.rezultat_prihodnost = None
        self.tip_nav_cilja = None

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

        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.casovnik = self.create_timer(0.2, self.zanka)

        self.osvezi_stanje(sili_objavo=True)

        self.get_logger().info('Behavior manager started.')

    def razdalja(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def objavi_stanje(self, stanje, sili_objavo=False):
        if stanje == self.trenutno_stanje and not sili_objavo:
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

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.zadnja_poza_robota = msg.pose.pose

    def shrani_trenutno_pozo(self):
        if self.zadnja_poza_robota is None:
            self.get_logger().warn('Robot pose not available yet from /amcl_pose.')
            return False

        self.shranjena_poza = PoseStamped()
        self.shranjena_poza.header.frame_id = 'map'
        self.shranjena_poza.header.stamp = self.get_clock().now().to_msg()
        self.shranjena_poza.pose = self.zadnja_poza_robota
        return True

    def zanka(self):
        if not self.nav_streznik_pripravljen:
            if self.akcijski_odjemalec.wait_for_server(timeout_sec=0.01):
                self.nav_streznik_pripravljen = True
                self.get_logger().info('Behavior manager connected to navigate_to_pose.')
            return

        if self.rezultat_prihodnost is None:
            return

        if not self.rezultat_prihodnost.done():
            return

        try:
            rezultat = self.rezultat_prihodnost.result()
            status = rezultat.status
        except Exception as nap:
            self.get_logger().error(f'Navigation result error: {nap}')
            status = None

        tip_nav = self.tip_nav_cilja

        self.cilj_aktiven = False
        self.rocaj_cilja = None
        self.rezultat_prihodnost = None
        self.tip_nav_cilja = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            if tip_nav == 'approach_face':
                self.objavi_stanje('INTERACT_FACE')
                self.get_logger().info('Reached face target. Waiting for target_done.')
            elif tip_nav == 'approach_ring':
                self.objavi_stanje('INTERACT_RING')
                self.get_logger().info('Reached ring target. Waiting for target_done.')
            elif tip_nav == 'return_to_patrol':
                self.get_logger().info('Returned to saved patrol pose.')
                self.aktivna_tarca = None
                self.shranjena_poza = None
                self.osvezi_stanje()
            else:
                self.get_logger().info('Temporary navigation goal succeeded.')
        elif status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_CANCELING):
            self.get_logger().info(f'Temporary navigation cancelled: {tip_nav}')
            self.osvezi_stanje()
        else:
            self.get_logger().warn(f'Temporary navigation failed: {tip_nav}, status={status}')
            self.osvezi_stanje()

    def je_ze_obdelana(self, tip, x, y):
        for tar in self.obdelane_tarke:
            if tar['tip'] != tip:
                continue
            if self.razdalja(x, y, tar['x'], tar['y']) <= self.prag_ujemanja:
                return True
        return False

    def izracunaj_pristopno_tocko(self, x_tar, y_tar):
        if self.zadnja_poza_robota is None:
            return None

        x_rob = self.zadnja_poza_robota.position.x
        y_rob = self.zadnja_poza_robota.position.y

        dx = x_tar - x_rob
        dy = y_tar - y_rob
        raz = math.sqrt(dx * dx + dy * dy)

        if raz < 0.05:
            return None

        odmik = 0.6
        if raz <= odmik:
            x_cilj = x_rob
            y_cilj = y_rob
        else:
            faktor = (raz - odmik) / raz
            x_cilj = x_rob + dx * faktor
            y_cilj = y_rob + dy * faktor

        yaw = math.atan2(dy, dx)
        return x_cilj, y_cilj, yaw

    def poslji_nav_cilj(self, x, y, yaw, tip_nav_cilja):
        if not self.nav_streznik_pripravljen:
            self.get_logger().warn('navigate_to_pose server is not ready yet.')
            return False

        if self.cilj_aktiven or self.cakanje_na_sprejem or self.rezultat_prihodnost is not None:
            self.get_logger().warn('A temporary navigation goal is already active.')
            return False

        cilj = PoseStamped()
        cilj.header.frame_id = 'map'
        cilj.header.stamp = self.get_clock().now().to_msg()
        cilj.pose.position.x = float(x)
        cilj.pose.position.y = float(y)
        cilj.pose.orientation = yaw_v_kvaternion(float(yaw))

        sporocilo = NavigateToPose.Goal()
        sporocilo.pose = cilj

        self.tip_nav_cilja = tip_nav_cilja
        self.cakanje_na_sprejem = True

        prihodnost = self.akcijski_odjemalec.send_goal_async(sporocilo)
        prihodnost.add_done_callback(self.obdelaj_sprejem)

        self.get_logger().info(
            f'Sending temporary goal [{tip_nav_cilja}] x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )
        return True

    def obdelaj_sprejem(self, prihodnost):
        self.cakanje_na_sprejem = False

        try:
            rocaj_cilja = prihodnost.result()
        except Exception as nap:
            self.get_logger().error(f'Error while sending temporary goal: {nap}')
            self.tip_nav_cilja = None
            self.osvezi_stanje()
            return

        if not rocaj_cilja.accepted:
            self.get_logger().warn(f'Temporary goal rejected: {self.tip_nav_cilja}')
            self.tip_nav_cilja = None
            self.osvezi_stanje()
            return

        self.rocaj_cilja = rocaj_cilja
        self.cilj_aktiven = True
        self.rezultat_prihodnost = rocaj_cilja.get_result_async()

        self.get_logger().info(f'Temporary goal accepted: {self.tip_nav_cilja}')

    def preklici_zacasni_cilj(self):
        if self.rocaj_cilja is not None and self.cilj_aktiven:
            self.get_logger().info('Cancelling temporary navigation goal...')
            prihodnost = self.rocaj_cilja.cancel_goal_async()
            prihodnost.add_done_callback(self.obdelaj_preklic)

    def obdelaj_preklic(self, prihodnost):
        try:
            _ = prihodnost.result()
        except Exception as nap:
            self.get_logger().warn(f'Cancel temporary goal failed: {nap}')

        self.cilj_aktiven = False
        self.cakanje_na_sprejem = False
        self.rocaj_cilja = None
        self.rezultat_prihodnost = None
        self.tip_nav_cilja = None

    def osvezi_stanje(self, sili_objavo=False):
        if self.manualno_aktivno:
            self.objavi_patrol(False)
            self.objavi_stanje('MANUAL_CONTROL', sili_objavo)
            return

        if self.aktivna_tarca is not None:
            if self.tip_nav_cilja == 'return_to_patrol' or self.trenutno_stanje == 'RETURN_TO_PATROL':
                self.objavi_patrol(False)
                self.objavi_stanje('RETURN_TO_PATROL', sili_objavo)
                return

            if self.tip_nav_cilja == 'approach_face' or self.trenutno_stanje == 'INTERACT_FACE':
                self.objavi_patrol(False)
                if self.trenutno_stanje == 'INTERACT_FACE':
                    self.objavi_stanje('INTERACT_FACE', sili_objavo)
                else:
                    self.objavi_stanje('APPROACH_FACE', sili_objavo)
                return

            if self.tip_nav_cilja == 'approach_ring' or self.trenutno_stanje == 'INTERACT_RING':
                self.objavi_patrol(False)
                if self.trenutno_stanje == 'INTERACT_RING':
                    self.objavi_stanje('INTERACT_RING', sili_objavo)
                else:
                    self.objavi_stanje('APPROACH_RING', sili_objavo)
                return

            if self.aktivna_tarca['tip'] == 'face':
                self.objavi_patrol(False)
                self.objavi_stanje('APPROACH_FACE', sili_objavo)
                return

            if self.aktivna_tarca['tip'] == 'ring':
                self.objavi_patrol(False)
                self.objavi_stanje('APPROACH_RING', sili_objavo)
                return

        if self.patrola_zahtevana and not self.patrola_koncana:
            self.objavi_patrol(True)
            self.objavi_stanje('PATROL', sili_objavo)
            return

        self.objavi_patrol(False)
        self.objavi_stanje('IDLE', sili_objavo)

    def aktiviraj_tarco(self, tip, x, y, z):
        if self.aktivna_tarca is not None:
            return

        if self.zadnja_poza_robota is None:
            self.get_logger().warn('Ignoring target because /amcl_pose is not available yet.')
            return

        if not self.shrani_trenutno_pozo():
            return

        pristop = self.izracunaj_pristopno_tocko(x, y)
        if pristop is None:
            self.get_logger().warn('Could not compute an approach goal.')
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

        self.objavi_patrol(False)

        x_cilj, y_cilj, yaw = pristop
        tip_nav = 'approach_face' if tip == 'face' else 'approach_ring'

        if not self.poslji_nav_cilj(x_cilj, y_cilj, yaw, tip_nav):
            self.aktivna_tarca = None
            self.shranjena_poza = None
            self.osvezi_stanje()
            return

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

        self.aktiviraj_tarco('face', x, y, z)

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

        self.aktiviraj_tarco('ring', x, y, z)

    def target_done_callback(self, msg: Empty):
        if self.aktivna_tarca is None:
            self.get_logger().info('No active target to mark as done.')
            return

        if self.trenutno_stanje not in ('INTERACT_FACE', 'INTERACT_RING', 'APPROACH_FACE', 'APPROACH_RING'):
            self.get_logger().info('Target done received, but current state is not target handling.')
            return

        self.obdelane_tarke.append(self.aktivna_tarca)

        self.get_logger().info(
            f"Target completed -> type={self.aktivna_tarca['tip']}, "
            f"x={self.aktivna_tarca['x']:.2f}, y={self.aktivna_tarca['y']:.2f}"
        )

        if self.shranjena_poza is None:
            self.get_logger().warn('No saved patrol pose. Clearing target without return.')
            self.aktivna_tarca = None
            self.osvezi_stanje()
            return

        x = self.shranjena_poza.pose.position.x
        y = self.shranjena_poza.pose.position.y
        yaw = 2.0 * math.atan2(
            self.shranjena_poza.pose.orientation.z,
            self.shranjena_poza.pose.orientation.w
        )

        if self.cilj_aktiven:
            self.preklici_zacasni_cilj()

        if self.poslji_nav_cilj(x, y, yaw, 'return_to_patrol'):
            self.objavi_stanje('RETURN_TO_PATROL')
        else:
            self.get_logger().warn('Could not start return-to-patrol navigation.')
            self.aktivna_tarca = None
            self.shranjena_poza = None
            self.osvezi_stanje()

    def resume_patrol_callback(self, msg: Empty):
        if self.aktivna_tarca is not None:
            self.get_logger().info('Clearing active target and resuming patrol.')
        else:
            self.get_logger().info('No active target. Refreshing state.')

        if self.cilj_aktiven:
            self.preklici_zacasni_cilj()

        self.aktivna_tarca = None
        self.shranjena_poza = None
        self.osvezi_stanje()

    def manual_callback(self, msg: Bool):
        self.manualno_aktivno = msg.data

        if self.manualno_aktivno and self.cilj_aktiven:
            self.preklici_zacasni_cilj()

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