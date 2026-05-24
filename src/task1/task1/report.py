#!/usr/bin/env python3
import os
import re
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from fpdf import FPDF


REPORT_DIR_DEFAULT = os.path.expanduser("~/RINS_Project/reports")


class InspectionPDF(FPDF):
    def __init__(self, report_name):
        super().__init__()
        self.report_name = report_name
        self.set_auto_page_break(auto=True, margin=20)

    def header(self):
        pass

    def footer(self):
        self.set_y(-15)
        self.set_font("Helvetica", "I", 8)
        self.cell(0, 10, f"Page {self.page_no()}/{{nb}}", align="C")

    def section_title(self, title):
        self.set_font("Helvetica", "B", 14)
        self.set_x(self.l_margin)
        self.cell(0, 10, "", ln=True)
        self.set_x(self.l_margin)
        self.cell(0, 8, title, ln=True)
        self.set_draw_color(180, 180, 180)
        self.set_x(self.l_margin)
        self.line(self.l_margin, self.get_y(), self.w - self.r_margin, self.get_y())
        self.ln(4)

    @property
    def _text_w(self):
        return self.w - self.l_margin - self.r_margin

    def subsection(self, text):
        self.set_font("Helvetica", "B", 11)
        self.set_x(self.l_margin)
        self.cell(self._text_w, 7, text, ln=True)

    def body(self, text):
        self.set_font("Helvetica", "", 10)
        self.set_x(self.l_margin)
        self.multi_cell(self._text_w, 5.5, text)

    def table_header(self, columns, widths):
        self.set_font("Helvetica", "B", 10)
        self.set_fill_color(230, 230, 230)
        for col, w in zip(columns, widths):
            self.cell(w, 8, col, border=1, fill=True, align="C")
        self.ln()

    def table_row(self, cells, widths):
        self.set_font("Helvetica", "", 10)
        for cell_text, w in zip(cells, widths):
            self.cell(w, 7, str(cell_text), border=1, align="C")
        self.ln()


class ReportManager(Node):
    def __init__(self):
        super().__init__("report_manager")

        self.declare_parameter("report_dir", REPORT_DIR_DEFAULT)
        report_dir = self.get_parameter("report_dir").value

        self.pdf_dir = os.path.join(report_dir, "pdf")
        self.img_barrels_dir = os.path.join(report_dir, "img", "barrels")
        self.img_defects_dir = os.path.join(report_dir, "img", "defects")

        self.report_counter = self._get_last_report_index() + 1
        self._barrel_image_seq = 0
        self._defect_image_seq = 0

        self.tiles_per_station = {}
        self._current_station = None
        self._latest_warped = None
        self._latest_heatmap = None
        self.bridge = CvBridge()

        self._ensure_dirs()

        self.cmd_sub = self.create_subscription(
            String, "/report_commands", self._cmd_callback, 10
        )
        self.station_sub = self.create_subscription(
            String, "/inspector_station", self._station_callback, 10
        )
        self.tile_status_sub = self.create_subscription(
            String, "/tile_status", self._tile_status_callback, 10
        )
        self.tile_classification_sub = self.create_subscription(
            String, "/tile_classification", self._tile_classification_callback, 10
        )
        self.tile_warped_sub = self.create_subscription(
            Image, "/tile_warped", self._tile_warped_callback, 10
        )
        self.tile_heatmap_sub = self.create_subscription(
            Image, "/tile_heatmap", self._tile_heatmap_callback, 10
        )

        self.get_logger().info(
            f"ReportManager ready — reports: {self.pdf_dir}"
        )

    def _ensure_dirs(self):
        for d in (self.pdf_dir, self.img_barrels_dir, self.img_defects_dir):
            os.makedirs(d, exist_ok=True)

    def _cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        noinc = False
        if cmd.startswith("make"):
            noinc = "--no-increment" in cmd
            self._make_report(noinc)
        elif cmd == "clear":
            self._clear_reports()
        else:
            self.get_logger().warn(f"Unknown report command: '{cmd}'")

    def _make_report(self, noinc=False):
        self._ensure_dirs()
        report_name = f"report{self.report_counter:02d}"
        pdf_path = os.path.join(self.pdf_dir, f"{report_name}.pdf")
        self._barrel_image_seq = 0
        self._defect_image_seq = 0

        self._generate_pdf(pdf_path, report_name)
        self._write_markdown(report_name)

        self.get_logger().info(f"Report {report_name} generated")
        if not noinc:
            self.report_counter += 1

    def _clear_reports(self):
        for d in (self.pdf_dir, self.img_barrels_dir, self.img_defects_dir):
            if os.path.isdir(d):
                for fname in os.listdir(d):
                    fpath = os.path.join(d, fname)
                    if os.path.isfile(fpath):
                        try:
                            os.remove(fpath)
                        except OSError:
                            pass

        self.report_counter = 0
        self._barrel_image_seq = 0
        self._defect_image_seq = 0
        self.tiles_per_station = {}
        self._current_station = None
        self._latest_warped = None
        self._latest_heatmap = None
        self._ensure_dirs()
        self.get_logger().info("Reports cleared")

    def _gen_barrel_image_name(self):
        seq = self._barrel_image_seq
        self._barrel_image_seq += 1
        return f"barrel{self.report_counter:02d}_{seq:04d}.jpg"

    def _get_last_report_index(self):
        self._ensure_dirs()
        pattern = re.compile(r"^report(\d+)\.pdf$")
        max_idx = -1
        for fname in os.listdir(self.pdf_dir):
            m = pattern.match(fname)
            if m:
                idx = int(m.group(1))
                if idx > max_idx:
                    max_idx = idx
        if max_idx >= 0:
            self.get_logger().info(
                f"Resuming from last report: report{max_idx:02d}"
            )
        return max_idx

    def _get_current_tiles(self):
        if self._current_station is None:
            return None
        if self._current_station not in self.tiles_per_station:
            self.tiles_per_station[self._current_station] = []
        return self.tiles_per_station[self._current_station]

    def _station_callback(self, msg: String):
        station = msg.data.strip().lower()
        if station in ("green", "red"):
            self._current_station = station
            if station not in self.tiles_per_station:
                self.tiles_per_station[station] = []
            self.get_logger().info(f"Station set: {station}")

    def _tile_status_callback(self, msg: String):
        data = msg.data.strip()
        if data != "TILE_FOUND":
            return
        tiles = self._get_current_tiles()
        if tiles is None:
            return
        tile = {
            "id": len(tiles) + 1,
            "status": None,
            "texture_img": None,
            "heatmap_img": None,
        }
        tiles.append(tile)
        self.get_logger().info(
            f"Tile #{tile['id']} [{self._current_station}] detected"
        )

    def _tile_warped_callback(self, msg: Image):
        try:
            self._latest_warped = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _tile_heatmap_callback(self, msg: Image):
        try:
            self._latest_heatmap = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _tile_classification_callback(self, msg: String):
        label = msg.data.strip()
        tiles = self._get_current_tiles()
        if not tiles:
            return

        tile = tiles[-1]
        tile["status"] = label
        self.get_logger().info(
            f"Tile #{tile['id']} [{self._current_station}]: {label}"
        )

        if label != "DEFECT":
            return
        station = self._current_station or "unknown"

        seq = self._defect_image_seq
        self._defect_image_seq += 1

        if self._latest_warped is not None:
            self._ensure_dirs()
            tex_name = f"defect{self.report_counter:02d}_{seq:04d}_{station}.png"
            tex_path = os.path.join(self.img_defects_dir, tex_name)
            cv2.imwrite(tex_path, self._latest_warped)
            tile["texture_img"] = tex_path

        if self._latest_heatmap is not None:
            self._ensure_dirs()
            hm_name = f"heatmap{self.report_counter:02d}_{seq:04d}_{station}.png"
            hm_path = os.path.join(self.img_defects_dir, hm_name)
            cv2.imwrite(hm_path, self._latest_heatmap)
            tile["heatmap_img"] = hm_path

    def _generate_pdf(self, pdf_path, report_name):
        pdf = InspectionPDF(report_name)
        pdf.alias_nb_pages()
        pdf.add_page()

        today = datetime.now().strftime("%d. %m. %Y")

        pdf.set_font("Helvetica", "B", 20)
        pdf.cell(0, 12, "Inspection Report", ln=True, align="C")
        pdf.ln(4)

        pdf.set_font("Helvetica", "", 10)
        pdf.cell(0, 7, f"Date: {today}", ln=True)
        pdf.cell(0, 7, f"Robot: BigChungus", ln=True)
        pdf.cell(0, 7, f"Report: {report_name}", ln=True)
        pdf.ln(6)

        pdf.section_title("Task: Ring Counting")
        pdf.subsection("Requested by: Not implemented yet")
        pdf.ln(2)
        pdf.body("Results:")
        pdf.body("- Total number of rings detected:  0")
        pdf.body("- Detected colors:")
        pdf.body("    {color1}: 0")
        pdf.body("    {color2}: 0")
        pdf.ln(4)

        pdf.add_page()
        pdf.section_title("Task: Barrel Inspection")
        pdf.subsection("Requested by: Not implemented yet")
        pdf.ln(2)
        pdf.body("Results:")
        pdf.body("- Total number of barrels inspected:  0")
        pdf.ln(2)
        cols = ("Barrel ID", "Colour", "Orientation", "Leak detected")
        widths = (28, 28, 40, 36)
        pdf.table_header(cols, widths)
        pdf.table_row(("-", "-", "-", "-"), widths)
        pdf.ln(4)

        pdf.add_page()
        pdf.section_title("Task: Anomaly Detection")

        if not self.tiles_per_station:
            pdf.set_font("Helvetica", "I", 10)
            pdf.set_x(pdf.l_margin)
            pdf.multi_cell(pdf._text_w, 7, "**Anomaly inspection was not requested.**")
            pdf.output(pdf_path)
            self.get_logger().info(f"PDF saved: {pdf_path}")
            return

        first = True
        for station in ("green", "red"):
            tiles = self.tiles_per_station.get(station, [])
            if not tiles:
                continue

            if not first:
                pdf.add_page()
                pdf.section_title("Task: Anomaly Detection")
            first = False

            pdf.subsection("Requested by: Not implemented yet")
            pdf.subsection(f"Station: {station.capitalize()}")
            pdf.ln(2)
            pdf.subsection("Results:")
            pdf.ln(1)

            total = len(tiles)
            ok_count = sum(1 for t in tiles if t["status"] == "OK")
            defect_count = sum(1 for t in tiles if t["status"] == "DEFECT")
            pdf.body(f"- Total number of tiles inspected:  {total}")
            pdf.body(f"- OK: {ok_count}")
            pdf.body(f"- DEFECT: {defect_count}")
            pdf.ln(2)
            cols2 = ("Tile ID", "Status")
            widths2 = (30, 40)
            pdf.table_header(cols2, widths2)
            for tile in tiles:
                status = tile["status"] or "-"
                pdf.table_row((str(tile["id"]), status), widths2)
            pdf.ln(2)

            defect_tiles = [t for t in tiles if t["status"] == "DEFECT"]
            if defect_tiles:
                pdf.set_font("Helvetica", "I", 9)
                pdf.cell(0, 6, "Anomaly Inspection Visuals:", ln=True)
                pdf.ln(1)

            img_w = 30
            gap = 1
            pair_gap = 4
            row_y = pdf.get_y()
            i = 0
            while i < len(defect_tiles):
                t0 = defect_tiles[i]
                t1 = defect_tiles[i + 1] if i + 1 < len(defect_tiles) else None

                col0_x = pdf.l_margin
                col1_x = pdf.l_margin + img_w * 2 + gap + pair_gap

                pdf.set_font("Helvetica", "", 8)
                pdf.set_xy(col0_x, row_y)
                pdf.cell(img_w * 2 + gap, 5, f"ID: {t0['id']}", align="C")
                if t1:
                    pdf.set_xy(col1_x, row_y)
                    pdf.cell(img_w * 2 + gap, 5, f"ID: {t1['id']}", align="C")

                img_y = row_y + 6
                for tile, px in ((t0, col0_x), (t1, col1_x)) if t1 else ((t0, col0_x),):
                    tex = tile.get("texture_img")
                    hm = tile.get("heatmap_img")
                    if tex and os.path.isfile(tex):
                        pdf.image(tex, x=px, y=img_y, w=img_w)
                    if hm and os.path.isfile(hm):
                        pdf.image(hm, x=px + img_w + gap, y=img_y, w=img_w)

                row_y = img_y + 36
                pdf.set_y(row_y)
                i += 2

        pdf.output(pdf_path)
        self.get_logger().info(f"PDF saved: {pdf_path}")

    def _write_markdown(self, report_name):
        md_path = os.path.join(self.pdf_dir, f"{report_name}.md")
        today = datetime.now().strftime("%d. %m. %Y")

        text = f"""# Inspection Report

**Date:** {today}
**Robot:** BigChungus

---

## Task: Ring Counting
**Requested by:** Not implemented yet

### Results:
- **Total number of rings detected:** 0
- **Detected colors:**
  - {{color1}}: 0
  - {{color2}}: 0

---

## Task: Barrel Inspection
**Requested by:** Not implemented yet

### Results:
- **Total number of barrels inspected:** 0

| Barrel ID | Colour | Orientation | Leak detected |
| :---: | :--- | :--- | :---: |
| - | - | - | - |

---

## Task: Anomaly Detection

"""

        if not self.tiles_per_station:
            text += "**Anomaly inspection was not requested.**\n"
        else:
            for station in ("green", "red"):
                tiles = self.tiles_per_station.get(station, [])
                if not tiles:
                    continue

                total = len(tiles)
                ok_count = sum(1 for t in tiles if t["status"] == "OK")
                defect_count = sum(1 for t in tiles if t["status"] == "DEFECT")

                tile_rows = "\n".join(
                    f"| {t['id']} | {t['status'] or '-'} |"
                    for t in tiles
                )

                text += f"""

**Requested by:** Not implemented yet
**Station:** {station.capitalize()}

### Results:
- **Total number of tiles inspected:** {total}
- **OK:** {ok_count}
- **DEFECT:** {defect_count}

| Tile ID | Status |
| :---: | :---: |
{tile_rows}
"""

                for t in tiles:
                    if t["status"] != "DEFECT":
                        continue
                    tex = t.get("texture_img")
                    hm = t.get("heatmap_img")
                    text += f"\nID {t['id']} Anomaly Inspection Visuals:\n\n"
                    if tex:
                        text += f"![Defect texture]({tex}) "
                    if hm:
                        text += f"![Defect heatmap]({hm}) "
                    text += "\n\n"

        with open(md_path, "w", encoding="utf-8") as f:
            f.write(text)
        self.get_logger().info(f"Markdown saved: {md_path}")


def main(args=None):
    rclpy.init(args=args)
    node = ReportManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
