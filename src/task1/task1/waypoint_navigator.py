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
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, Empty


def yaw_v_kvaternion(kot):
    kvaternion = Quaternion()
    kvaternion.z = math.sin(kot / 2.0)
    kvaternion.w = math.cos(kot / 2.0)
    return kvaternion


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

        self.rocaj_cilja = None
        self.rezultat_prihodnost = None
        self.pause_until = None  # nanoseconds timestamp; robot dwells after arrival

        qos_lat = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub_pat = self.create_subscription(
            Bool,
            '/patrol_enabled',
            self.patrol_callback,
            qos_lat
        )

        self.pub_kon = self.create_publisher(
            Bool,
            '/patrol_finished',
            qos_lat
        )

        self.pub_group_end = self.create_publisher(Empty, '/patrol_group_end', 10)

        self.casovnik = self.create_timer(0.2, self.zanka)

        self.get_logger().info(f'Nalozenih waypointov: {len(self.seznam_tock)}')
        self.objavi_koncanost(False)

    def nalozi_tocke(self):
        pot_paketa = get_package_share_directory('task1')
        pot_yaml = os.path.join(pot_paketa, 'config', 'waypoints.yaml')

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
            self.get_logger().info('Cancelling current patrol goal...')
            handle = self.rocaj_cilja
            prihodnost = handle.cancel_goal_async()
            prihodnost.add_done_callback(lambda f: self.obdelaj_preklic(f, handle))

    def obdelaj_preklic(self, prihodnost, cancelled_handle):
        try:
            _ = prihodnost.result()
        except Exception as nap:
            self.get_logger().warn(f'Cancel request failed: {nap}')

        # Only wipe state if we haven't already moved on to a new goal.
        # If a new goal was sent before this callback fired, rocaj_cilja will
        # no longer be the same handle we cancelled — don't touch anything.
        if self.rocaj_cilja is cancelled_handle or self.rocaj_cilja is None:
            self.cilj_aktiven = False
            self.cakanje_na_sprejem = False
            self.rocaj_cilja = None
            self.rezultat_prihodnost = None

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
            self.pause_until = None
            # Discard a completed result that arrived while patrol was off.
            # Without this, a stale STATUS_SUCCEEDED would advance indeks_tocke
            # on the next tick after patrol resumes, causing the robot to skip
            # the first waypoint of the next group.
            if self.rezultat_prihodnost is not None and self.rezultat_prihodnost.done():
                self.rezultat_prihodnost = None
                self.cilj_aktiven = False
                self.rocaj_cilja = None
            return

        if not self.prej_patro_om:
            self.get_logger().info('Resuming patrol...')
            self.prej_patro_om = True

        # Dwell after arrival before sending next goal
        if self.pause_until is not None:
            if self.get_clock().now().nanoseconds >= self.pause_until:
                self.pause_until = None
                self.poslji_naslednjo_tocko()
            return

        if self.indeks_tocke >= len(self.seznam_tock):
            self.get_logger().info('Vse tocke so opravljene.')
            self.koncan = True
            self.objavi_koncanost(True)
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

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Tocka {self.indeks_tocke + 1} dosezena.')
            completed_point = self.seznam_tock[self.indeks_tocke]
            pause = float(completed_point.get('pause', 0.0))
            self.indeks_tocke += 1

            if self.indeks_tocke >= len(self.seznam_tock):
                self.get_logger().info('Vse tocke so opravljene.')
                self.koncan = True
                self.objavi_koncanost(True)
                self.pub_group_end.publish(Empty())
            else:
                next_point = self.seznam_tock[self.indeks_tocke]
                group_ended = (
                    float(next_point['x']) != float(completed_point['x']) or
                    float(next_point['y']) != float(completed_point['y'])
                )
                if group_ended:
                    self.pub_group_end.publish(Empty())
                    if pause > 0.0:
                        self.pause_until = self.get_clock().now().nanoseconds + int(pause * 1e9)
                        self.get_logger().info(f'Group done. Dwelling {pause:.1f}s.')
                    # Do NOT call poslji_naslednjo_tocko() here — let the next
                    # zanka() tick send the goal after behavior_manager has had a
                    # chance to pause patrol for any deferred barrel targets.
                elif pause > 0.0:
                    self.pause_until = self.get_clock().now().nanoseconds + int(pause * 1e9)
                    self.get_logger().info(f'Dwelling {pause:.1f}s before next waypoint.')
                elif self.patrol_omogocen:
                    self.poslji_naslednjo_tocko()

        elif status in (
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_CANCELING,
        ):
            self.get_logger().info('Patrol goal cancelled.')

        else:
            self.get_logger().error(f'Cilj ni uspel. Status: {status}')

    def poslji_naslednjo_tocko(self):
        if self.indeks_tocke >= len(self.seznam_tock):
            self.koncan = True
            self.objavi_koncanost(True)
            return

        if not self.patrol_omogocen:
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
            self.get_logger().error(f'Tocka {self.indeks_tocke + 1} zavrnjena.')
            return

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