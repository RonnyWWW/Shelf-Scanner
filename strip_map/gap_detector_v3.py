#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Int64
from cv_bridge import CvBridge


class GapDetector(Node):
    def __init__(self):
        super().__init__('gap_detector')

        self.declare_parameter('process_every_n_frames', 2)
        self.declare_parameter('min_gap_width_m', 0.18)
        self.declare_parameter('pixels_per_meter', 180.0)
        self.declare_parameter('min_gap_height_px', 70)
        self.declare_parameter('min_gap_area_px', 300)
        self.declare_parameter('base_depth_delta', 45.0)
        self.declare_parameter('height_split_ratio', 0.55)
        self.declare_parameter('min_shelf_strength', 0.45)
        self.declare_parameter('shelf_window', 12)
        self.declare_parameter('active_width', 220)
        self.declare_parameter('new_track_min_x', 470.0)

        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('max_missed_frames_candidate', 3)
        self.declare_parameter('max_missed_frames_confirmed', 8)
        self.declare_parameter('match_x_tol_px', 45)
        self.declare_parameter('match_y_tol_px', 30)
        self.declare_parameter('match_w_tol_px', 40)
        self.declare_parameter('match_h_tol_px', 50)

        self.declare_parameter('angle_start_deg', 42.0)
        self.declare_parameter('angle_end_deg', 100.0)

        self.process_every_n_frames = int(self.get_parameter('process_every_n_frames').value)
        self.min_gap_width_m = float(self.get_parameter('min_gap_width_m').value)
        self.pixels_per_meter = float(self.get_parameter('pixels_per_meter').value)
        self.min_gap_height_px = int(self.get_parameter('min_gap_height_px').value)
        self.min_gap_area_px = int(self.get_parameter('min_gap_area_px').value)
        self.base_depth_delta = float(self.get_parameter('base_depth_delta').value)
        self.height_split_ratio = float(self.get_parameter('height_split_ratio').value)
        self.min_shelf_strength = float(self.get_parameter('min_shelf_strength').value)
        self.shelf_window = int(self.get_parameter('shelf_window').value)
        self.active_width = int(self.get_parameter('active_width').value)
        self.new_track_min_x = float(self.get_parameter('new_track_min_x').value)

        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.max_missed_frames_candidate = int(self.get_parameter('max_missed_frames_candidate').value)
        self.max_missed_frames_confirmed = int(self.get_parameter('max_missed_frames_confirmed').value)
        self.match_x_tol = int(self.get_parameter('match_x_tol_px').value)
        self.match_y_tol = int(self.get_parameter('match_y_tol_px').value)
        self.match_w_tol = int(self.get_parameter('match_w_tol_px').value)
        self.match_h_tol = int(self.get_parameter('match_h_tol_px').value)

        self.angle_start_deg = float(self.get_parameter('angle_start_deg').value)
        self.angle_end_deg = float(self.get_parameter('angle_end_deg').value)

        self.image_sub = self.create_subscription(Image, '/strip/image', self.image_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/sick_tim_5xx/scan', self.scan_cb, 10)
        self.column_sub = self.create_subscription(Int64, '/strip/column_count', self.column_count_cb, 10)

        self.gaps_pub = self.create_publisher(Image, '/strip/gaps', 10)
        self.trigger_pub = self.create_publisher(String, '/gap_events/trigger', 10)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.latest_distance = 0.38
        self.last_column_count = None

        self.next_gap_id = 1
        self.active_tracks = []

        self.get_logger().info(
            f"✅ GapDetector v3 running\n"
            f"   confirm={self.confirm_frames}, active_width={self.active_width}, min_gap_width={self.min_gap_width_m:.2f}m\n"
            f"   new_track_min_x={self.new_track_min_x:.1f}"
        )

    def column_count_cb(self, msg: Int64):
        if self.last_column_count is None:
            self.last_column_count = int(msg.data)
            return

        delta = int(msg.data) - self.last_column_count
        self.last_column_count = int(msg.data)
        if delta <= 0:
            return

        for track in self.active_tracks:
            track['x0'] -= delta
            track['x1'] -= delta
            track['cx'] -= delta

    def scan_cb(self, msg: LaserScan):
        angles_full = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges), dtype=np.float32)
        mask = (
            angles_full > np.deg2rad(self.angle_start_deg)
        ) & (
            angles_full < np.deg2rad(self.angle_end_deg)
        )
        ranges = np.array(msg.ranges, dtype=np.float32)[mask]
        ranges = np.nan_to_num(ranges, nan=np.inf, posinf=np.inf, neginf=np.inf)
        valid = np.isfinite(ranges) & (ranges > msg.range_min + 0.03) & (ranges < 5.0)
        if np.any(valid):
            self.latest_distance = float(np.median(ranges[valid]))

    def _is_match(self, track, det):
        return (
            track['shelf_idx'] == det['shelf_idx'] and
            abs(track['cx'] - det['cx']) <= self.match_x_tol and
            abs(track['cy'] - det['cy']) <= self.match_y_tol
        )

    def _update_tracks(self, detections, dist):
        published_events = []
        matched_track_ids = set()

        for det in detections:
            best_track = None
            best_score = 1e9
            for track in self.active_tracks:
                if track['id'] in matched_track_ids:
                    continue
                if not self._is_match(track, det):
                    continue
                score = (
                    abs(track['cx'] - det['cx']) +
                    abs(track['cy'] - det['cy']) +
                    0.25 * abs(track['w'] - det['w']) +
                    0.15 * abs(track['h'] - det['h'])
                )
                if score < best_score:
                    best_score = score
                    best_track = track

            if best_track is None:
                if det['cx'] < self.new_track_min_x:
                    continue
                best_track = {
                    'id': self.next_gap_id,
                    'seen_count': 0,
                    'missed_count': 0,
                    'confirmed': False,
                    'published': False,
                }
                self.next_gap_id += 1
                self.active_tracks.append(best_track)

            best_track.update(det)
            best_track['seen_count'] += 1
            best_track['missed_count'] = 0
            best_track['last_distance_m'] = dist
            matched_track_ids.add(best_track['id'])

            if not best_track['confirmed'] and best_track['seen_count'] >= self.confirm_frames:
                best_track['confirmed'] = True
            if best_track['confirmed'] and not best_track['published']:
                published_events.append(best_track.copy())
                best_track['published'] = True

        survivors = []
        for track in self.active_tracks:
            if track['id'] not in matched_track_ids:
                track['missed_count'] += 1
            max_missed = self.max_missed_frames_confirmed if track['confirmed'] else self.max_missed_frames_candidate
            if track['missed_count'] <= max_missed and track['x1'] > 0:
                survivors.append(track)
        self.active_tracks = survivors
        return published_events

    def image_cb(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % max(self.process_every_n_frames, 1) != 0:
            return

        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            h, w = gray.shape
            display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            min_gap_width_px = max(int(self.min_gap_width_m * self.pixels_per_meter), 6)
            min_gap_height = self.min_gap_height_px
            min_gap_area = self.min_gap_area_px
            depth_delta_thresh = self.base_depth_delta

            recent_cols = gray[:, max(0, w - self.shelf_window):w]
            column_profile = np.mean(recent_cols, axis=1)
            col_blur = cv2.GaussianBlur(column_profile.astype(np.float32), (9, 9), 0)
            col_norm = cv2.normalize(col_blur, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            peak_thresh = float(col_norm.mean()) + self.min_shelf_strength * float(col_norm.std())
            bright_rows = np.where(col_norm > peak_thresh)[0]

            shelf_fronts = []
            if len(bright_rows) > 0:
                start = bright_rows[0]
                prev = bright_rows[0]
                bands = []
                for r in bright_rows[1:]:
                    if r == prev + 1:
                        prev = r
                    else:
                        bands.append((start, prev))
                        start = r
                        prev = r
                bands.append((start, prev))
                shelf_fronts = sorted([(s + e) // 2 for s, e in bands])
            if not shelf_fronts:
                shelf_fronts = [h // 2]

            shelf_intervals = []
            for i, fr in enumerate(shelf_fronts):
                top = fr + 3
                bottom = shelf_fronts[i + 1] - 3 if i < len(shelf_fronts) - 1 else h
                if bottom > top + 5:
                    shelf_intervals.append((fr, top, bottom))

            for idx, sf in enumerate(shelf_fronts):
                cv2.line(display, (0, sf), (w - 1, sf), (0, 255, 255), 1)
                cv2.putText(display, f"Shelf {idx+1}", (10, max(sf - 8, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            active_width = min(self.active_width, w)
            active_start_x = w - active_width
            overlay = display.copy()
            cv2.rectangle(overlay, (active_start_x, 0), (w - 1, h - 1), (255, 0, 0), -1)
            display = cv2.addWeighted(overlay, 0.12, display, 0.88, 0)
            cv2.rectangle(display, (active_start_x, 0), (w - 1, h - 1), (255, 0, 0), 2)

            kernel = np.ones((3, 3), np.uint8)
            current_detections = []

            for shelf_idx, (front_row, top, bottom) in enumerate(shelf_intervals, start=1):
                baseline_row = gray[front_row, :].astype(np.float32)
                region = gray[top:bottom, :].astype(np.float32)
                baseline = np.tile(baseline_row, (bottom - top, 1))
                depth_delta = baseline - region
                shelf_mask = (depth_delta > depth_delta_thresh).astype(np.uint8) * 255
                shelf_mask[:, :active_start_x] = 0
                shelf_mask = cv2.morphologyEx(shelf_mask, cv2.MORPH_OPEN, kernel, iterations=1)
                contours, _ = cv2.findContours(shelf_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    x, y, w_box, h_box = cv2.boundingRect(contour)
                    area = cv2.contourArea(contour)
                    if w_box < min_gap_width_px or h_box < min_gap_height or area < min_gap_area:
                        continue

                    submask = shelf_mask[y:y+h_box, x:x+w_box]
                    mask_bool = submask > 0
                    any_fg = mask_bool.any(axis=0)
                    first_y = np.argmax(mask_bool, axis=0)
                    last_y = mask_bool.shape[0] - 1 - np.argmax(mask_bool[::-1, :], axis=0)
                    col_heights = np.where(any_fg, last_y - first_y + 1, 0).astype(np.int32)

                    max_col_height = int(col_heights.max()) if col_heights.size > 0 else 0
                    if max_col_height < min_gap_height:
                        continue
                    strong_h = int(max(min_gap_height, max_col_height * self.height_split_ratio))

                    gap_runs = []
                    in_run = False
                    sx = 0
                    for cx, ch in enumerate(col_heights.tolist()):
                        if ch >= strong_h:
                            if not in_run:
                                in_run = True
                                sx = cx
                        elif in_run:
                            gap_runs.append((sx, cx - 1))
                            in_run = False
                    if in_run:
                        gap_runs.append((sx, w_box - 1))

                    for gx0, gx1 in gap_runs:
                        gap_w = gx1 - gx0 + 1
                        if gap_w < min_gap_width_px:
                            continue
                        sub_cols = submask[:, gx0:gx1+1] > 0
                        if not np.any(sub_cols):
                            continue
                        row_has_fg = np.any(sub_cols, axis=1)
                        gy0 = int(np.argmax(row_has_fg))
                        gy1 = int(len(row_has_fg) - 1 - np.argmax(row_has_fg[::-1]))
                        x0 = x + gx0
                        x1 = x + gx1
                        y0 = top + y + gy0
                        y1 = top + y + gy1
                        gap_h = y1 - y0 + 1
                        if gap_h < min_gap_height:
                            continue
                        current_detections.append({
                            'x0': x0, 'y0': y0, 'x1': x1, 'y1': y1,
                            'cx': (x0 + x1) // 2, 'cy': (y0 + y1) // 2,
                            'w': x1 - x0 + 1, 'h': gap_h, 'shelf_idx': shelf_idx,
                        })

            published_events = self._update_tracks(current_detections, self.latest_distance)

            for track in self.active_tracks:
                color = (0, 255, 255) if not track['confirmed'] else (0, 0, 255)
                cv2.rectangle(display, (track['x0'], track['y0']), (track['x1'], track['y1']), color, 2)
                label = f"Gap {track['id']} S{track['shelf_idx']}"
                if not track['confirmed']:
                    label += f" ? ({track['seen_count']}/{self.confirm_frames})"
                cv2.putText(display, label, (track['x0'], max(track['y0'] - 5, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            confirmed_count = sum(1 for t in self.active_tracks if t['confirmed'])
            cv2.putText(display, f"Distance: {self.latest_distance:.2f}m", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display, f"Confirmed: {confirmed_count}  Candidates: {len(self.active_tracks)}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if confirmed_count else (255, 255, 255), 2)

            out_msg = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            out_msg.header = msg.header
            self.gaps_pub.publish(out_msg)

            for event in published_events:
                payload = json.dumps({
                    'event_type': 'new_gap',
                    'gap_id': int(event['id']),
                    'gap_count': 1,
                    'distance_m': round(float(event['last_distance_m']), 3),
                    'shelf_idx': int(event['shelf_idx']),
                    'x_center': int(event['cx']),
                    'y_center': int(event['cy']),
                    'width_px': int(event['w']),
                    'height_px': int(event['h']),
                })
                self.trigger_pub.publish(String(data=payload))

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
