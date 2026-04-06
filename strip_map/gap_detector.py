#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class GapDetector(Node):
    def __init__(self):
        super().__init__('gap_detector')

        # ============ FRAME SKIP PARAMETER ============
        self.declare_parameter('process_every_n_frames', 1)
        self.process_every_n_frames = (
            self.get_parameter('process_every_n_frames')
                .get_parameter_value().integer_value
        )
        self.frame_count = 0
        # ==============================================

        # ------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            '/strip/image',
            self.image_cb,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/sick_tim_5xx/scan',
            self.scan_cb,
            10
        )

        # ------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------
        self.gaps_pub    = self.create_publisher(Image,  '/strip/gaps',         10)
        self.trigger_pub = self.create_publisher(String, '/gap_events/trigger', 10)

        self.bridge = CvBridge()

        # ------------------------------------------------------------
        # Tunable base parameters
        # ------------------------------------------------------------
        self.min_gap_width_m = 0.10   # 10 cm
        self.pixels_per_meter = 200.0

        self.base_min_gap_width   = int(self.min_gap_width_m * self.pixels_per_meter)  # 20 px
        self.base_min_gap_height  = 80
        self.base_min_gap_area    = int(self.base_min_gap_height * self.base_min_gap_width * 0.375)  # 375
        self.base_depth_delta     = 40.0
        self.height_split_ratio   = 0.5
        self.min_shelf_strength   = 0.3
        self.shelf_window         = 10

        # Distance scaling
        self.ref_distance    = 1.0
        self.latest_distance = None

        # Only detect gaps in NEW (rightmost) region
        self.active_width = 250

        # ------------------------------------------------------------
        # Temporal tracking state
        # ------------------------------------------------------------
        self.next_gap_id = 1
        self.active_tracks = []

        self.confirm_frames = 3
        self.max_missed_frames = 10
        self.match_x_tol = 60
        self.match_y_tol = 40
        self.match_w_tol = 50

        self.get_logger().info(
            f"✅ GapDetector running — publishing to /strip/gaps "
            f"and /gap_events/trigger\n"
            f"   Processing every {self.process_every_n_frames} frame(s)\n"
            f"   Min gap width: {self.min_gap_width_m*100:.0f} cm "
            f"({self.base_min_gap_width} px at {self.pixels_per_meter:.0f} px/m)\n"
            f"   Temporal tracking: confirm={self.confirm_frames}, max_missed={self.max_missed_frames}"
        )

    # ------------------------------------------------------------------
    # LiDAR callback: estimate current distance to shelf
    # ------------------------------------------------------------------
    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=np.inf, posinf=np.inf)

        valid = (ranges > msg.range_min + 0.05) & (ranges < 10.0)
        if not np.any(valid):
            return

        n      = len(ranges)
        center = n // 2
        span   = n // 10
        lo     = max(0, center - span)
        hi     = min(n, center + span)

        subset = ranges[lo:hi]
        subset = subset[np.isfinite(subset)]
        if subset.size == 0:
            return

        self.latest_distance = float(np.median(subset))

    def _is_match(self, track, det):
        return (
            track["shelf_idx"] == det["shelf_idx"] and
            abs(track["cx"] - det["cx"]) <= self.match_x_tol and
            abs(track["cy"] - det["cy"]) <= self.match_y_tol and
            abs(track["w"]  - det["w"])  <= self.match_w_tol
        )

    def _update_tracks(self, detections, dist, scale):
        published_events = []
        matched_track_ids = set()

        for det in detections:
            best_track = None
            best_score = 1e9

            for track in self.active_tracks:
                if track["id"] in matched_track_ids:
                    continue
                if not self._is_match(track, det):
                    continue

                score = abs(track["cx"] - det["cx"]) + abs(track["cy"] - det["cy"])
                if score < best_score:
                    best_score = score
                    best_track = track

            if best_track is not None:
                best_track["x0"] = det["x0"]
                best_track["y0"] = det["y0"]
                best_track["x1"] = det["x1"]
                best_track["y1"] = det["y1"]
                best_track["cx"] = det["cx"]
                best_track["cy"] = det["cy"]
                best_track["w"] = det["w"]
                best_track["h"] = det["h"]
                best_track["shelf_idx"] = det["shelf_idx"]
                best_track["seen_count"] += 1
                best_track["missed_count"] = 0
                best_track["last_distance_m"] = dist
                best_track["last_scale"] = scale
                matched_track_ids.add(best_track["id"])

                if (not best_track["confirmed"] and
                        best_track["seen_count"] >= self.confirm_frames):
                    best_track["confirmed"] = True

                if best_track["confirmed"] and not best_track["published"]:
                    published_events.append(best_track.copy())
                    best_track["published"] = True

            else:
                new_track = {
                    "id": self.next_gap_id,
                    "x0": det["x0"],
                    "y0": det["y0"],
                    "x1": det["x1"],
                    "y1": det["y1"],
                    "cx": det["cx"],
                    "cy": det["cy"],
                    "w": det["w"],
                    "h": det["h"],
                    "shelf_idx": det["shelf_idx"],
                    "seen_count": 1,
                    "missed_count": 0,
                    "confirmed": False,
                    "published": False,
                    "last_distance_m": dist,
                    "last_scale": scale,
                }
                self.active_tracks.append(new_track)
                matched_track_ids.add(new_track["id"])
                self.next_gap_id += 1

        survivors = []
        for track in self.active_tracks:
            if track["id"] not in matched_track_ids:
                track["missed_count"] += 1
            if track["missed_count"] <= self.max_missed_frames:
                survivors.append(track)

        self.active_tracks = survivors
        return published_events

    # ------------------------------------------------------------------
    # Main image callback
    # ------------------------------------------------------------------
    def image_cb(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            h, w = gray.shape

            if self.latest_distance is None:
                dist = self.ref_distance
            else:
                dist = max(self.latest_distance, 0.3)

            scale = float(np.clip(self.ref_distance / dist, 0.4, 1.6))

            min_gap_height     = max(int(self.base_min_gap_height * scale), 10)
            min_gap_width      = max(int(self.base_min_gap_width  * scale), 5)
            min_gap_area       = max(int(self.base_min_gap_area   * (scale ** 2)), 80)
            depth_delta_thresh = self.base_depth_delta * scale

            # 1) Detect shelf fronts
            W              = self.shelf_window
            recent_cols    = gray[:, max(0, w - W):w]
            column_profile = np.mean(recent_cols, axis=1)

            col_blur = cv2.GaussianBlur(column_profile, (9, 9), 0)
            col_norm = cv2.normalize(col_blur, None, 0, 255,
                                     cv2.NORM_MINMAX).astype(np.uint8)

            col_mean    = float(col_norm.mean())
            col_std     = float(col_norm.std())
            peak_thresh = col_mean + self.min_shelf_strength * col_std

            bright_rows = np.where(col_norm > peak_thresh)[0]

            shelf_fronts = []
            if len(bright_rows) > 0:
                bands = []
                start = bright_rows[0]
                prev  = bright_rows[0]
                for r in bright_rows[1:]:
                    if r == prev + 1:
                        prev = r
                    else:
                        bands.append((start, prev))
                        start = r
                        prev  = r
                bands.append((start, prev))
                shelf_fronts = sorted([(s + e) // 2 for s, e in bands])

            if len(shelf_fronts) == 0:
                shelf_fronts = [h // 2]

            kernel = np.ones((3, 3), np.uint8)

            # 2) Shelf intervals
            shelf_fronts    = sorted(shelf_fronts)
            shelf_intervals = []
            for i, fr in enumerate(shelf_fronts):
                if i < len(shelf_fronts) - 1:
                    top    = fr + 3
                    bottom = shelf_fronts[i + 1] - 3
                else:
                    top    = fr + 3
                    bottom = h
                if bottom > top + 5:
                    shelf_intervals.append((fr, top, bottom))

            # 3) Detect gaps + build display image
            display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            for idx, sf in enumerate(shelf_fronts):
                cv2.line(display, (0, sf), (w - 1, sf), (0, 255, 255), 1)
                cv2.putText(display, f"Shelf {idx+1}",
                            (10, max(sf - 8, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1)

            active_width   = min(self.active_width, w)
            active_start_x = w - active_width

            overlay = display.copy()
            cv2.rectangle(overlay, (active_start_x, 0), (w - 1, h - 1),
                          (255, 0, 0), -1)
            alpha   = 0.15
            display = cv2.addWeighted(overlay, alpha, display, 1 - alpha, 0)
            cv2.rectangle(display, (active_start_x, 0), (w - 1, h - 1),
                          (255, 0, 0), 2)

            current_detections = []

            for shelf_idx, (front_row, top, bottom) in enumerate(shelf_intervals):
                baseline_row = gray[front_row, :].astype(np.float32)
                region       = gray[top:bottom, :].astype(np.float32)

                baseline    = np.tile(baseline_row, (bottom - top, 1))
                depth_delta = baseline - region

                shelf_mask = (depth_delta > depth_delta_thresh).astype(np.uint8) * 255

                if active_start_x > 0:
                    shelf_mask[:, :active_start_x] = 0

                shelf_mask = cv2.morphologyEx(
                    shelf_mask, cv2.MORPH_OPEN, kernel, iterations=1
                )

                contours, _ = cv2.findContours(
                    shelf_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                for contour in contours:
                    x, y, w_box, h_box = cv2.boundingRect(contour)
                    area = cv2.contourArea(contour)

                    if x + w_box <= active_start_x:
                        continue
                    if w_box < min_gap_width or h_box < min_gap_height or area < min_gap_area:
                        continue

                    submask = shelf_mask[y:y + h_box, x:x + w_box]

                    col_heights = []
                    for cx in range(w_box):
                        col = submask[:, cx]
                        ys  = np.where(col > 0)[0]
                        col_heights.append(ys[-1] - ys[0] + 1 if len(ys) > 0 else 0)

                    max_col_height = max(col_heights) if col_heights else 0
                    if max_col_height < min_gap_height:
                        continue

                    strong_h = int(max(min_gap_height,
                                       max_col_height * self.height_split_ratio))

                    gap_runs = []
                    in_run   = False
                    sx       = 0
                    for cx, ch in enumerate(col_heights):
                        if ch >= strong_h:
                            if not in_run:
                                in_run = True
                                sx     = cx
                        else:
                            if in_run:
                                gap_runs.append((sx, cx - 1))
                                in_run = False
                    if in_run:
                        gap_runs.append((sx, w_box - 1))

                    for gx0, gx1 in gap_runs:
                        gap_w = gx1 - gx0 + 1
                        if gap_w < min_gap_width:
                            continue

                        sub_cols = submask[:, gx0:gx1 + 1]
                        ys = np.where(sub_cols > 0)[0]
                        if len(ys) == 0:
                            continue

                        gy0 = int(ys[0])
                        gy1 = int(ys[-1])

                        x0    = x + gx0
                        x1    = x + gx1
                        y0    = top + y + gy0
                        y1    = top + y + gy1
                        gap_h = y1 - y0 + 1

                        if gap_h < min_gap_height:
                            continue

                        det = {
                            "x0": x0,
                            "y0": y0,
                            "x1": x1,
                            "y1": y1,
                            "cx": (x0 + x1) // 2,
                            "cy": (y0 + y1) // 2,
                            "w": x1 - x0 + 1,
                            "h": gap_h,
                            "shelf_idx": shelf_idx + 1,
                        }
                        current_detections.append(det)

            published_events = self._update_tracks(current_detections, dist, scale)

            candidate_count = len(self.active_tracks)
            confirmed_count = sum(1 for t in self.active_tracks if t["confirmed"])

            for track in self.active_tracks:
                color = (0, 255, 255) if not track["confirmed"] else (0, 0, 255)
                cv2.rectangle(display,
                              (track["x0"], track["y0"]),
                              (track["x1"], track["y1"]),
                              color, 2)

                label = f"Gap {track['id']} S{track['shelf_idx']}"
                if not track["confirmed"]:
                    label += f" ? ({track['seen_count']}/{self.confirm_frames})"

                cv2.putText(display, label,
                            (track["x0"], max(track["y0"] - 5, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, color, 1)

            cv2.putText(display,
                        f"Distance: {dist:.2f} m  Scale: {scale:.2f}",
                        (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)

            status_color = (0, 0, 255) if confirmed_count > 0 else (255, 255, 255)
            cv2.putText(display,
                        f"Confirmed: {confirmed_count}  Candidates: {candidate_count}",
                        (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, status_color, 2)

            out_msg        = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            out_msg.header = msg.header
            self.gaps_pub.publish(out_msg)

            for event in published_events:
                payload = json.dumps({
                    "gap_id": event["id"],
                    "gap_count": 1,
                    "distance_m": round(event["last_distance_m"], 3),
                    "scale": round(event["last_scale"], 3),
                    "shelf_idx": event["shelf_idx"],
                    "x_center": event["cx"],
                    "y_center": event["cy"],
                    "width_px": event["w"],
                    "height_px": event["h"],
                })
                self.trigger_pub.publish(String(data=payload))
                self.get_logger().info(
                    f"📢 New confirmed gap published → {payload}"
                )

            self.get_logger().info(
                f"🟡 dist={dist:.2f} | fronts={len(shelf_fronts)} | "
                f"detections={len(current_detections)} | confirmed={confirmed_count} | "
                f"candidates={candidate_count} | scale={scale:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"Error in image_cb: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GapDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
