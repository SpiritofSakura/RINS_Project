#!/usr/bin/env python3
import json
import os
import re
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
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
        self.declare_parameter("noqr", False)
        report_dir = self.get_parameter("report_dir").value

        self.pdf_dir = os.path.join(report_dir, "pdf")
        self.img_barrels_dir = os.path.join(report_dir, "img", "barrels")
        self.img_defects_dir = os.path.join(report_dir, "img", "defects")

        self.report_counter = self._get_last_report_index() + 1
        self._barrel_image_seq = 0
        self._defect_image_seq = 0

        self.tiles_per_station = {}
        self._current_station = None
        self.rings_requested = False
        self.ring_counts = {}
        self.barrels_requested = False
        self.barrel_data = []
        self.defect_stations = set()
        self.requested_by = "Not implemented yet"
        self._noqr = self.get_parameter("noqr").value
        if self._noqr:
            self.get_logger().info("noqr mode: showing all section headers")
        self._warped_cache = {}
        self._heatmap_cache = {}
        self._pending_defect = None
        self.bridge = CvBridge()

        self._ensure_dirs()

        self.ring_sub = self.create_subscription(
            Marker, "/detected_ring_locations", self._ring_callback, 10
        )
        self.barrel_sub = self.create_subscription(
            Marker, "/detected_cylinder_locations", self._barrel_callback, 10
        )
        self.barrel_result_sub = self.create_subscription(
            String, "/barrel_inspection_result", self._barrel_result_callback, 10
        )
        self.qr_sub = self.create_subscription(
            String, "/qr", self._qr_callback, 10
        )
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
        self.create_timer(0.1, self._check_defect_timeout)

    def _ensure_dirs(self):
        for d in (self.pdf_dir, self.img_barrels_dir, self.img_defects_dir):
            os.makedirs(d, exist_ok=True)

    def _qr_callback(self, msg: String):
        lower = msg.data.strip().lower()
        if lower == "rings":
            self.rings_requested = True
            self.get_logger().info("QR requested: rings")
        elif lower == "barrels":
            self.barrels_requested = True
            self.get_logger().info("QR requested: barrels")
        elif lower.startswith("defects "):
            color = lower.split(" ", 1)[1]
            self.defect_stations.add(color)
            self.get_logger().info(f"QR requested: defects for {color}")
        elif lower.startswith("person "):
            name = msg.data.strip()[7:]
            self.requested_by = name
            self.get_logger().info(f"Task requested by: {name}")

    def _marker_to_color(self, marker: Marker):
        r, g, b = round(marker.color.r, 2), round(marker.color.g, 2), round(marker.color.b, 2)
        colors = {
            (1.0, 0.0, 0.0): "red",
            (0.0, 1.0, 0.0): "green",
            (0.0, 0.4, 1.0): "blue",
            (1.0, 1.0, 0.0): "yellow",
            (1.0, 0.5, 0.0): "orange",
            (0.6, 0.0, 1.0): "purple",
            (0.45, 0.22, 0.08): "brown",
            (0.1, 0.1, 0.1): "black",
        }
        best, best_dist = "unknown", float("inf")
        for (cr, cg, cb), name in colors.items():
            d = abs(r - cr) + abs(g - cg) + abs(b - cb)
            if d < best_dist:
                best_dist = d
                best = name
        return best

    def _ring_callback(self, msg: Marker):
        if msg.ns not in ("ring_confirmed", "ring_actionable"):
            return
        color = self._marker_to_color(msg)
        if not hasattr(self, '_seen_ring_ids'):
            self._seen_ring_ids = set()
        if msg.id in self._seen_ring_ids:
            return
        self._seen_ring_ids.add(msg.id)
        self.ring_counts[color] = self.ring_counts.get(color, 0) + 1

    def _barrel_callback(self, msg: Marker):
        if msg.ns not in ("barrel_confirmed", "cylinder_confirmed"):
            return
        color = self._marker_to_color(msg)
        orientation = msg.text if msg.text in ("horizontal", "vertical") else "unknown"
        barrel_id = msg.id
        if not any(b["id"] == barrel_id for b in self.barrel_data):
            self.barrel_data.append({
                "id": barrel_id,
                "colour": color,
                "orientation": orientation,
                "leak_detected": "?",
            })
            self.get_logger().info(f"Barrel #{barrel_id}: {color} {orientation}")

    def _barrel_result_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            bid = data.get("barrel_id")
            leak = data.get("leak_detected")
            for b in self.barrel_data:
                if b["id"] == bid:
                    b["leak_detected"] = "Yes" if leak else "No" if leak is False else "?"
                    break
        except Exception:
            pass

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
        import shutil
        report_dir = os.path.dirname(self.pdf_dir)
        if os.path.isdir(report_dir):
            shutil.rmtree(report_dir, ignore_errors=True)
        self._ensure_dirs()

        self.report_counter = 0
        self._barrel_image_seq = 0
        self._defect_image_seq = 0
        self.tiles_per_station = {}
        self._current_station = None
        self._warped_cache = {}
        self._heatmap_cache = {}
        self._pending_defect = None
        self._seen_ring_ids = set()
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
            self.defect_stations.add(station)
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
            self._warped_cache[msg.header.frame_id] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass
        self._trim_caches()
        self._try_complete_defect()

    def _tile_heatmap_callback(self, msg: Image):
        try:
            self._heatmap_cache[msg.header.frame_id] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass
        self._trim_caches()
        self._try_complete_defect()

    def _trim_caches(self):
        for cache in (self._warped_cache, self._heatmap_cache):
            if len(cache) > 20:
                for old in sorted(cache.keys()) [:-20]:
                    del cache[old]

    def _tile_classification_callback(self, msg: String):
        raw = msg.data.strip()
        if ":" in raw:
            label, tile_id = raw.rsplit(":", 1)
        else:
            label = raw
            tile_id = ""
        tiles = self._get_current_tiles()
        if not tiles:
            return

        tile = tiles[-1]
        tile["status"] = label
        self.get_logger().info(
            f"Tile #{tile['id']} [{self._current_station}] tid={tile_id}: {label}"
        )

        if label != "DEFECT":
            return
        station = self._current_station or "unknown"

        seq = self._defect_image_seq
        self._defect_image_seq += 1

        self._ensure_dirs()
        tex_name = f"defect{self.report_counter:02d}_{seq:04d}_{station}.png"
        hm_name = f"heatmap{self.report_counter:02d}_{seq:04d}_{station}.png"
        tex_path = os.path.join(self.img_defects_dir, tex_name)
        hm_path = os.path.join(self.img_defects_dir, hm_name)

        saved_tex = False
        saved_hm = False

        if tile_id and tile_id in self._warped_cache:
            cv2.imwrite(tex_path, self._warped_cache[tile_id])
            tile["texture_img"] = tex_path
            saved_tex = True
        elif not tile_id and self._warped_cache:
            latest_key = sorted(self._warped_cache.keys())[-1]
            cv2.imwrite(tex_path, self._warped_cache[latest_key])
            tile["texture_img"] = tex_path
            saved_tex = True

        if tile_id and tile_id in self._heatmap_cache:
            cv2.imwrite(hm_path, self._heatmap_cache[tile_id])
            tile["heatmap_img"] = hm_path
            saved_hm = True
        elif not tile_id and self._heatmap_cache:
            latest_key = sorted(self._heatmap_cache.keys())[-1]
            cv2.imwrite(hm_path, self._heatmap_cache[latest_key])
            tile["heatmap_img"] = hm_path
            saved_hm = True

        if saved_tex and saved_hm:
            return

        self._pending_defect = {
            "tile": tile,
            "tile_id": tile_id,
            "since": self.get_clock().now(),
            "tex_path": tex_path,
            "hm_path": hm_path,
            "saved_tex": saved_tex,
            "saved_hm": saved_hm,
        }

    def _try_complete_defect(self):
        pending = self._pending_defect
        if pending is None:
            return

        tile = pending["tile"]
        tile_id = pending["tile_id"]

        if not pending["saved_tex"]:
            img = None
            if tile_id and tile_id in self._warped_cache:
                img = self._warped_cache[tile_id]
            elif not tile_id and self._warped_cache:
                img = self._warped_cache[sorted(self._warped_cache.keys())[-1]]
            if img is not None:
                cv2.imwrite(pending["tex_path"], img)
                tile["texture_img"] = pending["tex_path"]
                pending["saved_tex"] = True

        if not pending["saved_hm"]:
            img = None
            if tile_id and tile_id in self._heatmap_cache:
                img = self._heatmap_cache[tile_id]
            elif not tile_id and self._heatmap_cache:
                img = self._heatmap_cache[sorted(self._heatmap_cache.keys())[-1]]
            if img is not None:
                cv2.imwrite(pending["hm_path"], img)
                tile["heatmap_img"] = pending["hm_path"]
                pending["saved_hm"] = True

        if pending["saved_tex"] and pending["saved_hm"]:
            self._pending_defect = None

    def _check_defect_timeout(self):
        pending = self._pending_defect
        if pending is None:
            return
        elapsed = (self.get_clock().now() - pending["since"]).nanoseconds / 1e9
        if elapsed >= 2.0:
            self._pending_defect = None

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

        if self.rings_requested or sum(self.ring_counts.values()) > 0:
            total_rings = sum(self.ring_counts.values())
            pdf.section_title("Task: Ring Counting")
            pdf.subsection(f"Requested by: {self.requested_by}")
            pdf.ln(2)
            pdf.body("Results:")
            pdf.body(f"- Total number of rings detected:  {total_rings}")
            pdf.body("- Detected colors:")
            for color in ("red", "green", "blue", "yellow", "orange"):
                count = self.ring_counts.get(color, 0)
                if count > 0:
                    pdf.body(f"    {color}: {count}")
            if total_rings == 0:
                pdf.body("    (none)")
            pdf.ln(4)
        elif self._noqr:
            pdf.section_title("Task: Ring Counting")
            pdf.body("Task not requested")
            pdf.ln(4)

        if self.barrels_requested or len(self.barrel_data) > 0:
            pdf.add_page()
            pdf.section_title("Task: Barrel Inspection")
            pdf.subsection(f"Requested by: {self.requested_by}")
            pdf.ln(2)
            pdf.body("Results:")
            pdf.body(f"- Total number of barrels inspected:  {len(self.barrel_data)}")
            pdf.ln(2)
            cols = ("Barrel ID", "Colour", "Orientation", "Leak detected")
            widths = (28, 28, 40, 36)
            pdf.table_header(cols, widths)
            for b in self.barrel_data:
                pdf.table_row((str(b["id"]), b["colour"], b["orientation"], b["leak_detected"]), widths)
            if not self.barrel_data:
                pdf.table_row(("-", "-", "-", "-"), widths)
            pdf.ln(4)
        elif self._noqr:
            pdf.add_page()
            pdf.section_title("Task: Barrel Inspection")
            pdf.body("Task not requested")
            pdf.ln(4)

        if (self.barrels_requested or len(self.barrel_data) > 0) and os.path.isdir(self.img_barrels_dir):
            leak_images = sorted(
                [f for f in os.listdir(self.img_barrels_dir)
                 if f.startswith("leak_") and f.endswith(".jpg")],
                reverse=True,
            )
            if leak_images:
                pdf.set_font("Helvetica", "I", 9)
                pdf.cell(0, 6, "Leak Evidence:", ln=True)
                pdf.ln(1)
                img_w = 50
                gap = 4
                per_row = 3
                row_y = pdf.get_y()
                i = 0
                while i < len(leak_images):
                    row_images = leak_images[i:i + per_row]
                    col_positions = [
                        pdf.l_margin + j * (img_w + gap)
                        for j in range(len(row_images))
                    ]
                    pdf.set_font("Helvetica", "", 8)
                    for j, fname in enumerate(row_images):
                        seq = int(fname.replace("leak_", "").replace(".jpg", ""))
                        cx = col_positions[j]
                        pdf.set_xy(cx, row_y)
                        pdf.cell(img_w, 5, f"ID: {seq}", align="C")
                    img_y = row_y + 6
                    for j, fname in enumerate(row_images):
                        fpath = os.path.join(self.img_barrels_dir, fname)
                        if os.path.isfile(fpath):
                            pdf.image(fpath, x=col_positions[j], y=img_y, w=img_w)
                    row_y = img_y + img_w * 0.75 + 30
                    pdf.set_y(row_y)
                    i += per_row

        if not self.defect_stations and not any(
            tiles for tiles in self.tiles_per_station.values()
        ):
            if self._noqr:
                pdf.add_page()
                pdf.section_title("Task: Anomaly Detection")
                pdf.body("Task not requested")
                pdf.ln(4)
            pdf.output(pdf_path)
            self.get_logger().info(f"PDF saved: {pdf_path}")
            return

        pdf.add_page()
        pdf.section_title("Task: Anomaly Detection")

        first = True
        for station in sorted(self.defect_stations):
            tiles = self.tiles_per_station.get(station, [])
            if not tiles:
                continue

            if not first:
                pdf.add_page()
                pdf.section_title("Task: Anomaly Detection")
            first = False

            pdf.subsection(f"Requested by: {self.requested_by}")
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
"""

        if self.rings_requested or sum(self.ring_counts.values()) > 0:
            total_rings = sum(self.ring_counts.values())
            color_lines = "\n".join(
                f"  - {c}: {self.ring_counts.get(c, 0)}"
                for c in ("red", "green", "blue", "yellow", "orange")
                if self.ring_counts.get(c, 0) > 0
            ) or "  - (none)"
            text += f"""## Task: Ring Counting
**Requested by:** {self.requested_by}

### Results:
- **Total number of rings detected:** {total_rings}
- **Detected colors:**
{color_lines}

---
"""
        elif self._noqr:
            text += """## Task: Ring Counting
**Requested by:** Not implemented yet

### Results:
Task not requested

---
"""

        if self.barrels_requested or len(self.barrel_data) > 0:
            rows = "\n".join(
                f"| {b['id']} | {b['colour']} | {b['orientation']} | {b['leak_detected']} |"
                for b in self.barrel_data
            ) or "| - | - | - | - |"
            text += f"""## Task: Barrel Inspection
**Requested by:** {self.requested_by}

### Results:
- **Total number of barrels inspected:** {len(self.barrel_data)}

| Barrel ID | Colour | Orientation | Leak detected |
| :---: | :--- | :--- | :---: |
{rows}

---
"""
        elif self._noqr:
            text += """## Task: Barrel Inspection
**Requested by:** Not implemented yet

### Results:
Task not requested

---
"""

        if self.defect_stations or any(
            tiles for tiles in self.tiles_per_station.values()
        ):
            text += "## Task: Anomaly Detection\n\n"
            for station in sorted(self.defect_stations):
                tiles = self.tiles_per_station.get(station, [])

                total = len(tiles)
                ok_count = sum(1 for t in tiles if t["status"] == "OK")
                defect_count = sum(1 for t in tiles if t["status"] == "DEFECT")

                tile_rows = "\n".join(
                    f"| {t['id']} | {t['status'] or '-'} |"
                    for t in tiles
                )

                text += f"""

**Requested by:** {self.requested_by}
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
        elif self._noqr:
            text += """## Task: Anomaly Detection

**Requested by:** Not implemented yet

### Results:
Task not requested

---
"""

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
