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
        #   /strip/gaps          — annotated BGR image (existing, unchanged)
        #   /gap_events/trigger  — lightweight JSON string consumed by
        #                          firebase_uploader node
        # ------------------------------------------------------------
        self.gaps_pub    = self.create_publisher(Image,  '/strip/gaps',         10)
        self.trigger_pub = self.create_publisher(String, '/gap_events/trigger', 10)

        self.bridge = CvBridge()
        self.get_logger().info(
            f"✅ GapDetector running — publishing to /strip/gaps "
            f"and /gap_events/trigger\n"
            f"   Processing every {self.process_every_n_frames} frame(s)"
        )

        # ------------------------------------------------------------
        # Tunable base parameters
        # ------------------------------------------------------------
        self.base_min_gap_width   = 40
        self.base_min_gap_height  = 50
        self.base_min_gap_area    = 300
        self.base_depth_delta     = 40.0
        self.height_split_ratio   = 0.5
        self.min_shelf_strength   = 0.3
        self.shelf_window         = 10

        # Distance scaling
        self.ref_distance    = 1.0
        self.latest_distance = None

        # Only detect gaps in NEW (rightmost) region
        self.active_width = 250

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

    # ------------------------------------------------------------------
    # Main image callback
    # ------------------------------------------------------------------
    def image_cb(self, msg: Image):
        # ============ FRAME SKIP CHECK ============
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return
        # ==========================================

        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            h, w = gray.shape

            # ==============================================================
            # 0) Distance-based scaling
            # ==============================================================
            if self.latest_distance is None:
                dist = self.ref_distance
            else:
                dist = max(self.latest_distance, 0.3)

            scale = float(np.clip(self.ref_distance / dist, 0.4, 1.6))

            min_gap_height     = max(int(self.base_min_gap_height * scale), 10)
            min_gap_width      = max(int(self.base_min_gap_width  * scale), 5)
            min_gap_area       = max(int(self.base_min_gap_area   * (scale ** 2)), 80)
            depth_delta_thresh = self.base_depth_delta * scale

            # ==============================================================
            # 1) Detect shelf fronts (sliding window)
            # ==============================================================
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

            # ==============================================================
            # 2) Shelf intervals
            # ==============================================================
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

            # ==============================================================
            # 3) Detect gaps + build display image
            # ==============================================================
            display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            # Label shelves
            for idx, sf in enumerate(shelf_fronts):
                cv2.line(display, (0, sf), (w - 1, sf), (0, 255, 255), 1)
                cv2.putText(display, f"Shelf {idx+1}",
                            (10, max(sf - 8, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1)

            # Highlight active (rightmost) region
            active_width   = min(self.active_width, w)
            active_start_x = w - active_width

            overlay = display.copy()
            cv2.rectangle(overlay, (active_start_x, 0), (w - 1, h - 1),
                          (255, 0, 0), -1)
            alpha   = 0.15
            display = cv2.addWeighted(overlay, alpha, display, 1 - alpha, 0)
            cv2.rectangle(display, (active_start_x, 0), (w - 1, h - 1),
                          (255, 0, 0), 2)

            total_gaps = 0

            for (front_row, top, bottom) in shelf_intervals:
                baseline_row = gray[front_row, :].astype(np.float32)
                region       = gray[top:bottom, :].astype(np.float32)

                baseline    = np.tile(baseline_row, (bottom - top, 1))
                depth_delta = baseline - region

                shelf_mask = (depth_delta > depth_delta_thresh).astype(np.uint8) * 255

                if active_start_x > 0:
                    shelf_mask[:, :active_start_x] = 0

                shelf_mask = cv2.morphologyEx(shelf_mask, cv2.MORPH_OPEN,
                                              kernel, iterations=1)

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

                    max_col_height = max(col_heights)
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

                        cv2.rectangle(display, (x0, y0), (x1, y1), (0, 0, 255), 2)
                        total_gaps += 1

            # Distance / scale overlay
            cv2.putText(display,
                        f"Distance: {dist:.2f} m  Scale: {scale:.2f}",
                        (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)

            if total_gaps > 0:
                cv2.putText(display,
                            f"GAPS DETECTED: {total_gaps}",
                            (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 255), 2)
            else:
                cv2.putText(display, "No product gaps detected",
                            (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1)

            # ==============================================================
            # 4) Publish annotated image
            # ==============================================================
            out_msg        = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            out_msg.header = msg.header
            self.gaps_pub.publish(out_msg)

            # ==============================================================
            # 5) If gaps found, publish trigger for firebase_uploader node
            #    Payload is a small JSON string — no heavy data on this topic.
            # ==============================================================
            if total_gaps > 0:
                payload = json.dumps({
                    "gap_count":  total_gaps,
                    "distance_m": round(dist, 3),
                    "scale":      round(scale, 3),
                })
                self.trigger_pub.publish(String(data=payload))
                self.get_logger().info(
                    f"📢 Gap trigger published → {payload}"
                )

            self.get_logger().info(
                f"🟡 dist={dist:.2f} | fronts={len(shelf_fronts)} | "
                f"gaps={total_gaps} | scale={scale:.2f}"
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
