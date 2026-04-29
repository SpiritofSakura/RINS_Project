#!/usr/bin/env python3

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool


def yaw_v_kvaternion(kot):
    kvaternion = Quaternion()
    kvaternion.z = math.sin(kot / 2.0)
    kvaternion.w = math.cos(kot / 2.0)
    return kvaternion


def yaw_from_quaternion(q):
    return 2.0 * math.atan2(q.z, q.w)


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.akcijski_odjemalec = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.seznam_tock = self.nalozi_tocke()
        self.indeks_tocke = 0

        self.zacetek = False
        self.patrol_omogocen = False
        self.prej_patro_om = False
        self.cakanje_na_sprejem = False
        self.cilj_aktiven = False
        self.koncan = False

        # Initial scan delay: hold for 4s on first patrol start
        self.initial_scan_done = False
        self.initial_scan_start_time = None
        self.initial_scan_duration = 4.0

        self.rocaj_cilja = None
        self.rezultat_prihodnost = None

        # Stuck detection
        self.goal_sent_time = None
        self.stuck_timeout = 15.0
        self.consecutive_rejects = 0
        self.max_rejects = 10

        # Recovery: navigate 30cm behind robot using Nav2
        self.recovery_active = False
        self.recovery_backup_dist = 0.20
        self.recovery_sent_time = None
        self.recovery_timeout = 10.0
        self.current_pose = None

        qos_lat = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub_pat = self.create_subscription(
            Bool, '/patrol_enabled', self.patrol_callback, qos_lat
        )

        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10
        )

        self.pub_kon = self.create_publisher(Bool, '/patrol_finished', qos_lat)

        self.casovnik = self.create_timer(0.2, self.zanka)

        self.get_logger().info(f'Nalozenih waypointov: {len(self.seznam_tock)}')
        self.objavi_koncanost(False)

    def nalozi_tocke(self):
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        ime_datoteke = self.get_parameter('waypoints_file').get_parameter_value().string_value

        pot_paketa = get_package_share_directory('task1')
        pot_yaml = os.path.join(pot_paketa, 'config', ime_datoteke)

        if not os.path.exists(pot_yaml):
            self.get_logger().error(f'YAML datoteka ne obstaja: {pot_yaml}')
            return []

        with open(pot_yaml, 'r', encoding='utf-8') as dat:
            podatki = yaml.safe_load(dat)

        if podatki is None or 'waypoints' not in podatki:
            self.get_logger().error('V YAML manjka kljuc "waypoints".')
            return []

        seznam_tock = podatki['waypoints']

        if not isinstance(seznam_tock, list):
            self.get_logger().error('Kljuc "waypoints" mora vsebovati seznam.')
            return []

        return seznam_tock

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def objavi_koncanost(self, stanje):
        msg = Bool()
        msg.data = stanje
        self.pub_kon.publish(msg)

    def patrol_callback(self, msg: Bool):
        novo_stanje = msg.data

        if novo_stanje == self.patrol_omogocen:
            return

        self.patrol_omogocen = novo_stanje

        if self.patrol_omogocen:
            self.get_logger().info('Patrol enabled.')
            if self.koncan:
                self.get_logger().info('Patrol already completed.')
        else:
            self.get_logger().info('Patrol paused.')
            self.preklici_cilj()

    def preklici_cilj(self):
        if self.rocaj_cilja is not None and self.cilj_aktiven:
            self.get_logger().info('Cancelling current goal...')
            prihodnost = self.rocaj_cilja.cancel_goal_async()
            prihodnost.add_done_callback(self.obdelaj_preklic)

    def obdelaj_preklic(self, prihodnost):
        try:
            _ = prihodnost.result()
        except Exception as nap:
            self.get_logger().warn(f'Cancel request failed: {nap}')

        # Skip reset if recovery was already started — it owns state now
        if self.recovery_active or self.cakanje_na_sprejem:
            return

        self.cilj_aktiven = False
        self.rocaj_cilja = None
        self.rezultat_prihodnost = None
        self.goal_sent_time = None

    def start_recovery(self):
        if self.current_pose is None:
            self.get_logger().warn('No AMCL pose yet — skipping waypoint instead of recovery.')
            self.indeks_tocke += 1
            return

        yaw = yaw_from_quaternion(self.current_pose.orientation)
        backup_x = self.current_pose.position.x - self.recovery_backup_dist * math.cos(yaw)
        backup_y = self.current_pose.position.y - self.recovery_backup_dist * math.sin(yaw)

        self.get_logger().warn(
            f'Stuck — sending recovery goal 30cm back: ({backup_x:.2f}, {backup_y:.2f})'
        )

        self.recovery_active = True
        self.recovery_sent_time = self.get_clock().now()
        self.cilj_aktiven = False
        self.cakanje_na_sprejem = False
        self.rocaj_cilja = None
        self.rezultat_prihodnost = None
        self.goal_sent_time = None

        cilj = PoseStamped()
        cilj.header.frame_id = 'map'
        cilj.header.stamp = self.get_clock().now().to_msg()
        cilj.pose.position.x = backup_x
        cilj.pose.position.y = backup_y
        cilj.pose.orientation = yaw_v_kvaternion(yaw)

        sporocilo = NavigateToPose.Goal()
        sporocilo.pose = cilj

        self.cakanje_na_sprejem = True
        prihodnost = self.akcijski_odjemalec.send_goal_async(sporocilo)
        prihodnost.add_done_callback(self.obdelaj_sprejem_recovery)

    def obdelaj_sprejem_recovery(self, prihodnost):
        self.cakanje_na_sprejem = False

        try:
            rocaj_cilja = prihodnost.result()
        except Exception as nap:
            self.get_logger().error(f'Recovery goal error: {nap}')
            self.recovery_active = False
            return

        if not rocaj_cilja.accepted:
            self.get_logger().warn('Recovery goal rejected — skipping waypoint.')
            self.recovery_active = False
            self.recovery_sent_time = None
            self.indeks_tocke += 1
            return

        self.get_logger().info('Recovery goal accepted.')
        self.rocaj_cilja = rocaj_cilja
        self.cilj_aktiven = True
        self.rezultat_prihodnost = rocaj_cilja.get_result_async()

    def zanka(self):
        if len(self.seznam_tock) == 0:
            self.get_logger().error('Ni waypointov za izvajanje.')
            return

        if self.koncan:
            return

        if not self.zacetek:
            if not self.akcijski_odjemalec.wait_for_server(timeout_sec=0.2):
                self.get_logger().info('Waiting for navigate_to_pose server...')
                return
            self.zacetek = True

        if not self.patrol_omogocen:
            self.prej_patro_om = False
            return

        if not self.prej_patro_om:
            self.get_logger().info('Resuming patrol...')
            self.prej_patro_om = True
            if not self.initial_scan_done:
                self.initial_scan_start_time = self.get_clock().now()

        # Hold at start position for initial scan
        if not self.initial_scan_done and self.initial_scan_start_time is not None:
            elapsed = (self.get_clock().now() - self.initial_scan_start_time).nanoseconds / 1e9
            if elapsed < self.initial_scan_duration:
                return
            self.initial_scan_done = True
            self.get_logger().info('Initial scan complete. Starting patrol.')

        if self.indeks_tocke >= len(self.seznam_tock):
            self.get_logger().info('Vse tocke so opravljene.')
            self.koncan = True
            self.objavi_koncanost(True)
            return

        # Check if stuck on active patrol goal (not recovery)
        if self.cilj_aktiven and not self.recovery_active and self.goal_sent_time is not None:
            elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
            if elapsed > self.stuck_timeout:
                self.get_logger().warn(
                    f'Waypoint {self.indeks_tocke + 1} stuck for {elapsed:.1f}s — recovering.'
                )
                self.preklici_cilj()
                self.start_recovery()
                return

        # Check if recovery itself is stuck — hard-reset and skip waypoint
        if self.recovery_active and self.recovery_sent_time is not None:
            elapsed_rec = (self.get_clock().now() - self.recovery_sent_time).nanoseconds / 1e9
            if elapsed_rec > self.recovery_timeout:
                self.get_logger().warn(
                    f'Recovery stuck for {elapsed_rec:.1f}s — force-skipping waypoint {self.indeks_tocke + 1}.'
                )
                self.recovery_active = False
                self.recovery_sent_time = None
                self.cilj_aktiven = False
                self.cakanje_na_sprejem = False
                self.rocaj_cilja = None
                self.rezultat_prihodnost = None
                self.goal_sent_time = None
                self.indeks_tocke += 1
                return

        if not self.cakanje_na_sprejem and not self.cilj_aktiven and self.rezultat_prihodnost is None:
            self.poslji_naslednjo_tocko()
            return

        if self.rezultat_prihodnost is None:
            return

        if not self.rezultat_prihodnost.done():
            return

        rezultat = self.rezultat_prihodnost.result()
        status = rezultat.status

        self.cilj_aktiven = False
        self.rocaj_cilja = None
        self.rezultat_prihodnost = None
        self.goal_sent_time = None

        if self.recovery_active:
            # Recovery goal finished
            self.recovery_active = False
            self.recovery_sent_time = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Recovery succeeded — retrying waypoint.')
            else:
                self.get_logger().warn(f'Recovery failed (status {status}) — skipping waypoint.')
                self.indeks_tocke += 1
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Tocka {self.indeks_tocke + 1} dosezena.')
            self.consecutive_rejects = 0
            self.indeks_tocke += 1

            if self.indeks_tocke >= len(self.seznam_tock):
                self.get_logger().info('Vse tocke so opravljene.')
                self.koncan = True
                self.objavi_koncanost(True)
            elif self.patrol_omogocen:
                self.poslji_naslednjo_tocko()

        elif status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_CANCELING):
            self.get_logger().info('Patrol goal cancelled.')

        else:
            self.get_logger().error(f'Cilj ni uspel (status {status}) — recovering.')
            self.start_recovery()

    def poslji_naslednjo_tocko(self):
        if self.indeks_tocke >= len(self.seznam_tock):
            self.koncan = True
            self.objavi_koncanost(True)
            return

        if not self.patrol_omogocen:
            return

        if self.consecutive_rejects >= self.max_rejects:
            self.get_logger().warn(
                f'Waypoint {self.indeks_tocke + 1} rejected {self.consecutive_rejects} times — skipping.'
            )
            self.consecutive_rejects = 0
            self.indeks_tocke += 1
            return

        tocka = self.seznam_tock[self.indeks_tocke]

        cilj = PoseStamped()
        cilj.header.frame_id = 'map'
        cilj.header.stamp = self.get_clock().now().to_msg()
        cilj.pose.position.x = float(tocka['x'])
        cilj.pose.position.y = float(tocka['y'])

        yaw = float(tocka.get('yaw', 0.0))
        cilj.pose.orientation = yaw_v_kvaternion(yaw)

        sporocilo = NavigateToPose.Goal()
        sporocilo.pose = cilj

        self.get_logger().info(
            f"Posiljam tocko {self.indeks_tocke + 1}: "
            f"x={tocka['x']}, y={tocka['y']}, yaw={yaw}"
        )

        self.cakanje_na_sprejem = True
        self.goal_sent_time = self.get_clock().now()
        prihodnost = self.akcijski_odjemalec.send_goal_async(sporocilo)
        prihodnost.add_done_callback(self.obdelaj_sprejem)

    def obdelaj_sprejem(self, prihodnost):
        self.cakanje_na_sprejem = False

        try:
            rocaj_cilja = prihodnost.result()
        except Exception as nap:
            self.get_logger().error(f'Napaka pri sprejemu cilja: {nap}')
            return

        if not rocaj_cilja.accepted:
            self.consecutive_rejects += 1
            self.get_logger().error(
                f'Tocka {self.indeks_tocke + 1} zavrnjena ({self.consecutive_rejects}/{self.max_rejects}).'
            )
            self.goal_sent_time = None
            return

        self.consecutive_rejects = 0
        self.get_logger().info(f'Tocka {self.indeks_tocke + 1} sprejeta.')
        self.rocaj_cilja = rocaj_cilja
        self.cilj_aktiven = True
        self.rezultat_prihodnost = rocaj_cilja.get_result_async()


def main(args=None):
    rclpy.init(args=args)
    vozlisce = WaypointNavigator()
    rclpy.spin(vozlisce)
    vozlisce.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
