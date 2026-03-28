#!/usr/bin/env python3

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


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
        self.rezultat_prihodnost = None
        self.zacetek = False

        self.pub_stanje = self.create_publisher(String, '/robot_state', 10)

        self.casovnik = self.create_timer(0.5, self.zanka)

        self.get_logger().info(f'Nalozenih waypointov: {len(self.seznam_tock)}')


    def objavi_stanje(self, stanje):
        msg = String()
        msg.data = stanje
        self.pub_stanje.publish(msg)


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

    def zanka(self):
        if len(self.seznam_tock) == 0:
            self.get_logger().error('Ni waypointov za izvajanje.')
            rclpy.shutdown()
            return

        if not self.zacetek:
            if not self.akcijski_odjemalec.wait_for_server(timeout_sec=0.5):
                self.get_logger().info('Waiting for navigate_to_pose server...')
                return

            self.zacetek = True
            self.poslji_naslednjo_tocko()
            return

        if self.rezultat_prihodnost is None:
            return

        if not self.rezultat_prihodnost.done():
            return

        rezultat = self.rezultat_prihodnost.result()
        status = rezultat.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Tocka {self.indeks_tocke + 1} dosezena.')
            self.indeks_tocke += 1
            self.rezultat_prihodnost = None
            self.poslji_naslednjo_tocko()
        else:
            self.get_logger().error(f'Cilj ni uspel. Status: {status}')
            self.rezultat_prihodnost = None

    def poslji_naslednjo_tocko(self):
        if self.indeks_tocke >= len(self.seznam_tock):
            self.get_logger().info('Vse tocke so opravljene.')
            rclpy.shutdown()
            return

        tocka = self.seznam_tock[self.indeks_tocke]

        cilj = PoseStamped()
        cilj.header.frame_id = 'map'
        cilj.header.stamp = self.get_clock().now().to_msg()
        cilj.pose.position.x = float(tocka['x'])
        cilj.pose.position.y = float(tocka['y'])
        # Use default yaw of 0.0 if not specified
        yaw = float(tocka.get('yaw', 0.0))
        cilj.pose.orientation = yaw_v_kvaternion(yaw)

        sporocilo = NavigateToPose.Goal()
        sporocilo.pose = cilj

        self.get_logger().info(
            f"Posiljam tocko {self.indeks_tocke + 1}: "
            f"x={tocka['x']}, y={tocka['y']}, yaw={yaw}"
        )

        prihodnost = self.akcijski_odjemalec.send_goal_async(sporocilo)
        prihodnost.add_done_callback(self.obdelaj_sprejem)

    def obdelaj_sprejem(self, prihodnost):
        rocaj_cilja = prihodnost.result()

        if not rocaj_cilja.accepted:
            self.get_logger().error(f'Tocka {self.indeks_tocke + 1} zavrnjena.')
            return

        self.get_logger().info(f'Tocka {self.indeks_tocke + 1} sprejeta.')
        self.rezultat_prihodnost = rocaj_cilja.get_result_async()


def main(args=None):
    rclpy.init(args=args)
    vozlisce = WaypointNavigator()
    rclpy.spin(vozlisce)
    vozlisce.destroy_node()


if __name__ == '__main__':
    main()