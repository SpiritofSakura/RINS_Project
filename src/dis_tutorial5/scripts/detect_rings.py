#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2, math
import numpy as np
import tf2_ros

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Uvozimo potrebne QoS nastavitve
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RingDetector(Node):
    def __init__(self):
        super().__init__('ring_detector') # Popravljeno ime vozlišča

        # Pragovi za elipse - dodani omejitve velikosti
        self.ecc_thr = 300 
        self.ratio_thr = 1.5  # Razumna razmerja za krožne oblike
        self.center_thr = 10
        self.min_area = 200  # Minimalna površina elipse (približno)
        self.max_area = 15000  # Maksimalna površina elipse
        self.depth_threshold = 1.5  # Globina za detekcijo lebdečega obroča

        # Most za pretvorbo slik
        self.bridge = CvBridge()
        self.depth_image = None

        # Nastavimo QoS na BEST_EFFORT, da se ujema s simulatorjem (globina)
        depth_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Naročnine - uporabimo privzeto QoS (10) za RGB, ker je bolj zanesljivo
        self.image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.image_callback, 10)
        # Za globino pa nujno BEST_EFFORT
        self.depth_sub = self.create_subscription(Image, "/oakd/rgb/preview/depth", self.depth_callback, depth_qos)

        # Dodamo Publisher za markerje obročev, da jih vidiš v RViz
        self.marker_pub = self.create_publisher(Marker, "/ring_marker", 10)

        # Okna za vizualizacijo
        cv2.namedWindow("Binary Image", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detected rings", cv2.WINDOW_NORMAL)
        
        self.get_logger().info("Detektor obročev (z globino) zagnan!")

    def check_ring_variance(self, ellipse):
        """Preverimo varianco globine - pravi obroči imajo visoko varianco (luknja v sredini)"""
        cx, cy = int(ellipse[0][0]), int(ellipse[0][1])
        a, b = ellipse[1][0] / 2, ellipse[1][1] / 2
        
        # Preverimo če so koordinate veljalne
        if not (0 <= cy < self.depth_image.shape[0] and 0 <= cx < self.depth_image.shape[1]):
            return False
        
        # Preberemo globine na različnih točkah
        depths = []
        
        # Globina v sredini
        center_depth = self.depth_image[cy, cx]
        depths.append(center_depth)
        
        # Globine na robovih elipse (4 točke)
        for angle in [0, 90, 180, 270]:
            rad = np.radians(angle)
            px = int(cx + a * np.cos(rad))
            py = int(cy + b * np.sin(rad))
            
            if 0 <= py < self.depth_image.shape[0] and 0 <= px < self.depth_image.shape[1]:
                edge_depth = self.depth_image[py, px]
                depths.append(edge_depth)
        
        # Filtriramo neskončne vrednosti za analizo
        finite_depths = [d for d in depths if not np.isinf(d) and d > 0]
        
        # Pravi obroč ima neskončno globino v sredini in končno na robovih
        # Ali ima zelo malo končne globine (večina je neskončna)
        infinite_count = sum(1 for d in depths if np.isinf(d))
        
        if infinite_count >= 2:  # Vsaj 2 neskončni odčitka = verjetno luknja
            return True
        
        # Če so vse globine končne, preverimo varianco
        if len(finite_depths) >= 3:
            depth_array = np.array(finite_depths)
            variance = np.var(depth_array)
            std_dev = np.std(depth_array)
            
            # Pravi obroči imajo malo variance (vse točke so podobno oddaljene)
            # Obrazi imajo malo variance (flat surface), toda so zelo blizu
            # Zato preverimo tudi razdaljo - ring bi moral biti bolj oddaljen
            mean_depth = np.mean(depth_array)
            
            # Če je srednja globina manjša od 0.8m in variance majhna, je verjetno obraz/flat surface
            if mean_depth < 0.8 and std_dev < 0.3:
                return False
        
        return np.isinf(center_depth)

    def depth_callback(self, data):
        try:
            # Shranimo globinsko sliko kot 32-bitne float vrednosti (v metrih)
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Depth convert error: {e}")

    def image_callback(self, data):
        # Če še nimamo globinske slike, ne moremo filtrirati kock
        if self.depth_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"RGB convert error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 30)
        cv2.imshow("Binary Image", thresh)

        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        elps = []
        for cnt in contours:
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                e = ellipse[1]
                ratio = e[1]/e[0] if e[0] > 0 else 0
                area = np.pi * e[0] * e[1] / 4  # Približna površina elipse
                
                if (ratio <= self.ratio_thr and e[0] < self.ecc_thr and e[1] < self.ecc_thr and
                    self.min_area <= area <= self.max_area):
                    elps.append(ellipse)

        for ellipse in elps:
            # IMPLEMENTACIJA: Preverimo vsako elipso posamično
            # 1. Dobimo koordinate središča (cx, cy)
            cx, cy = int(ellipse[0][0]), int(ellipse[0][1])

            # 2. Preverimo, če so koordinate znotraj slike
            if 0 <= cy < self.depth_image.shape[0] and 0 <= cx < self.depth_image.shape[1]:
                # 3. Preverimo varianco globine (ali je to resničen obroč ali kaj drugega)
                if self.check_ring_variance(ellipse):
                    # To je lebdeči obroč
                    mid_depth = self.depth_image[cy, cx]
                    self.get_logger().info(f"Najden lebdeči obroč! Globina v sredini: {mid_depth:.2f}m")
                    cv2.ellipse(cv_image, ellipse, (0, 255, 0), 3) # Zelena barva za lebdečega
                    # Objavimo marker, da ga vidiš v RViz
                    self.publish_ring_marker(ellipse, mid_depth if not np.isinf(mid_depth) else 2.0)
                else:
                    # To je obroč na kocki, obraz ali lažni pozitiv
                    # self.get_logger().info(f"Ignoriram. Ni pravi obroč")
                    cv2.ellipse(cv_image, ellipse, (0, 0, 255), 1) # Rdeča tanjša črta za ignoriranega

        cv2.imshow("Detected rings", cv_image)
        cv2.waitKey(1)

    def publish_ring_marker(self, ellipse, depth):
        # Ustvarimo marker za RViz
        marker = Marker()
        # Nastavimo frame na kamero, ker so koordinate iz globinske slike
        marker.header.frame_id = "oakd_rgb_camera_optical_frame" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.id = 0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        # Marker bo RDEČ, da ga ločiš od zelenih obrazov
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Položaj: z je globina, x in y bi morala izračunati iz pikslov, 
        # a za test damo približno na sredino
        marker.pose.position.x = 0.0 
        marker.pose.position.y = 0.0
        marker.pose.position.z = float(depth)
        
        self.marker_pub.publish(marker)

def main():
    rclpy.init(args=None)
    node = RingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()