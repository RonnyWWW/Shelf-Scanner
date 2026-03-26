#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np

# ── Firebase / Storage ──────────────────────────────────────────────────────
import threading
import datetime
import tempfile
import os
import firebase_admin
from firebase_admin import credentials, firestore, storage as fb_storage

# ---------------------------------------------------------------------------
# Firebase initialisation
# Place your downloaded service-account JSON at the path below, OR set the
# environment variable  GOOGLE_APPLICATION_CREDENTIALS  to its path.
# Download from: Firebase Console → Project Settings → Service Accounts
#                → Generate new private key
# ---------------------------------------------------------------------------
_SERVICE_ACCOUNT_PATH = os.environ.get(
    "GOOGLE_APPLICATION_CREDENTIALS",
    os.path.expanduser("~/firebase_service_account.json")
)

_FIREBASE_STORAGE_BUCKET = "sick-lidar-3.firebasestorage.app"

def _init_firebase():
    """Initialise Firebase Admin SDK once."""
    if not firebase_admin._apps:
        cred = credentials.Certificate(_SERVICE_ACCOUNT_PATH)
        firebase_admin.initialize_app(cred, {
            "storageBucket": _FIREBASE_STORAGE_BUCKET
        })

try:
    _init_firebase()
    _FIREBASE_OK = True
except Exception as _fb_err:
    print(f"[GapDetector] ⚠️  Firebase init failed: {_fb_err}")
    print("              Upload will be skipped until credentials are available.")
    _FIREBASE_OK = False
# ────────────────────────────────────────────────────────────────────────────


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
        # Publisher
        # ------------------------------------------------------------
        self.publisher = self.create_publisher(Image, '/strip/gaps', 10)

        self.bridge = CvBridge()
        self.get_logger().info(
            f"✅ GapDetector (distance-scaled, depth-delta, multi-shelf, "
            f"right-side only + USB CAMERA + FIREBASE) running...\n"
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

        # ------------------------------------------------------------
        # USB Camera setup
        # Change USB_CAM_INDEX env var if the Pi has multiple cameras
        # ------------------------------------------------------------
        self._cam_index = int(os.environ.get("USB_CAM_INDEX", "0"))
        self._cap = cv2.VideoCapture(self._cam_index)
        if not self._cap.isOpened():
            self.get_logger().warn(
                f"⚠️  USB camera at index {self._cam_index} could not be opened. "
                "Camera capture will be skipped."
            )
        else:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
            self.get_logger().info(
                f"📷 USB camera opened at index {self._cam_index}"
            )

        # ------------------------------------------------------------
        # Upload throttle
        # A new upload is only allowed after COOLDOWN_SEC seconds.
        # Override with env var GAP_COOLDOWN_SEC.
        # ------------------------------------------------------------
        self._cooldown_sec  = float(os.environ.get("GAP_COOLDOWN_SEC", "5.0"))
        self._last_upload_t = 0.0
        self._upload_lock   = threading.Lock()

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
    # Capture a fresh frame from the USB camera
    # ------------------------------------------------------------------
    def _capture_usb_frame(self):
        """Return a BGR numpy image from the USB camera, or None on failure."""
        if not self._cap.isOpened():
            return None
        # Flush buffered frames so we get the most current image
        for _ in range(2):
            self._cap.grab()
        ret, frame = self._cap.read()
        if not ret or frame is None:
            self.get_logger().warn("📷 USB camera read() failed.")
            return None
        return frame

    # ------------------------------------------------------------------
    # Upload images + metadata to Firebase (runs in a background thread)
    # ------------------------------------------------------------------
    def _upload_to_firebase(self, camera_frame: np.ndarray,
                             lidar_display: np.ndarray,
                             gap_count: int,
                             dist: float):
        """
        Uploads to Firebase Storage and writes a Firestore document.

        Storage layout:
            gap_events/
                YYYYMMDD_HHMMSS_<us>/
                    camera.jpg      ← USB camera photo
                    lidar.jpg       ← annotated LiDAR strip image

        Firestore document  (collection: gap_events):
            {
              timestamp:   ISO-8601 string,
              gap_count:   int,
              distance_m:  float,
              camera_url:  public download URL,
              lidar_url:   public download URL,
            }
        """
        if not _FIREBASE_OK:
            return

        try:
            now    = datetime.datetime.utcnow()
            ts_str = now.strftime("%Y%m%d_%H%M%S_") + f"{now.microsecond:06d}"
            folder = f"gap_events/{ts_str}"

            bucket = fb_storage.bucket()
            db     = firestore.client()

            # ── Upload camera image ────────────────────────────────────────
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tf:
                cv2.imwrite(tf.name, camera_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                cam_blob = bucket.blob(f"{folder}/camera.jpg")
                cam_blob.upload_from_filename(tf.name, content_type="image/jpeg")
                cam_blob.make_public()
                camera_url = cam_blob.public_url
            os.unlink(tf.name)

            # ── Upload LiDAR annotated image ───────────────────────────────
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tf:
                cv2.imwrite(tf.name, lidar_display, [cv2.IMWRITE_JPEG_QUALITY, 85])
                lidar_blob = bucket.blob(f"{folder}/lidar.jpg")
                lidar_blob.upload_from_filename(tf.name, content_type="image/jpeg")
                lidar_blob.make_public()
                lidar_url = lidar_blob.public_url
            os.unlink(tf.name)

            # ── Write Firestore document ───────────────────────────────────
            db.collection("gap_events").add({
                "timestamp":  now.isoformat() + "Z",
                "gap_count":  gap_count,
                "distance_m": round(dist, 3),
                "camera_url": camera_url,
                "lidar_url":  lidar_url,
            })

            self.get_logger().info(
                f"☁️  Firebase upload OK → {folder}  "
                f"(gaps={gap_count}, dist={dist:.2f} m)"
            )

        except Exception as e:
            self.get_logger().error(f"Firebase upload failed: {e}")

    # ------------------------------------------------------------------
    # Trigger: capture camera + kick off upload (non-blocking)
    # ------------------------------------------------------------------
    def _trigger_camera_and_upload(self, lidar_display: np.ndarray,
                                   gap_count: int, dist: float):
        import time
        now = time.monotonic()

        with self._upload_lock:
            if now - self._last_upload_t < self._cooldown_sec:
                return          # still in cooldown window — skip
            self._last_upload_t = now

        camera_frame = self._capture_usb_frame()
        if camera_frame is None:
            self.get_logger().warn("📷 Skipping upload — no camera frame.")
            return

        # Deep-copy display so the background thread owns its own buffer
        lidar_copy = lidar_display.copy()

        threading.Thread(
            target=self._upload_to_firebase,
            args=(camera_frame, lidar_copy, gap_count, dist),
            daemon=True
        ).start()

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
            # 3) Shelf intervals
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
            # 4) Detect gaps
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

            # ==============================================================
            # 5) Gap-triggered camera capture + Firebase upload
            # ==============================================================
            if total_gaps > 0:
                cv2.putText(display,
                            f"GAPS DETECTED: {total_gaps}",
                            (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 255), 2)

                self._trigger_camera_and_upload(
                    lidar_display=display,
                    gap_count=total_gaps,
                    dist=dist
                )
            else:
                cv2.putText(display, "No product gaps detected",
                            (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1)

            # ==============================================================
            # Publish annotated frame
            # ==============================================================
            out_msg        = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            out_msg.header = msg.header
            self.publisher.publish(out_msg)

            self.get_logger().info(
                f"🟡 dist={dist:.2f} | fronts={len(shelf_fronts)} | "
                f"gaps={total_gaps} | scale={scale:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"Error in image_cb: {e}")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
            self.get_logger().info("📷 USB camera released.")
        super().destroy_node()


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
