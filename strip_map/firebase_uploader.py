#!/usr/bin/env python3
"""
firebase_uploader.py
────────────────────
Standalone ROS 2 node that:
  1. Subscribes to  /gap_events/trigger  (std_msgs/String, JSON payload)
     published by gap_detector when a shelf gap is confirmed.
  2. Captures a photo using rpicam-still (Pi CSI camera).
  3. Subscribes to  /strip/gaps  (sensor_msgs/Image) and latches the most
     recent annotated LiDAR frame to include in the upload.
  4. Uploads both images to Firebase Storage.
  5. Writes a Firestore document to the  gap_events  collection.

Topic interface
───────────────
  Subscribes:
    /gap_events/trigger   std_msgs/String   JSON: {gap_count, distance_m, scale}
    /strip/gaps           sensor_msgs/Image  annotated LiDAR strip (latched)

Firebase Storage layout
───────────────────────
  gap_events/
    YYYYMMDD_HHMMSS_<us>/
      camera.jpg    ← Pi CSI camera photo
      lidar.jpg     ← annotated LiDAR strip

Firestore document  (collection: gap_events)
────────────────────────────────────────────
  {
    timestamp:   ISO-8601 string,
    gap_count:   int,
    distance_m:  float,
    scale:       float,
    camera_url:  string,
    lidar_url:   string,
  }

Configuration (environment variables)
──────────────────────────────────────
  GOOGLE_APPLICATION_CREDENTIALS   path to service-account JSON
                                   (default: ~/firebase_service_account.json)
  GAP_COOLDOWN_SEC                 minimum seconds between uploads (default: 5)
"""

import json
import os
import datetime
import tempfile
import threading
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

import firebase_admin
from firebase_admin import credentials, firestore, storage as fb_storage

# ── Firebase initialisation ─────────────────────────────────────────────────
_SERVICE_ACCOUNT_PATH = os.environ.get(
    "GOOGLE_APPLICATION_CREDENTIALS",
    os.path.expanduser("~/firebase_service_account.json")
)
_FIREBASE_STORAGE_BUCKET = "sick-lidar-3.firebasestorage.app"


def _init_firebase():
    if not firebase_admin._apps:
        cred = credentials.Certificate(_SERVICE_ACCOUNT_PATH)
        firebase_admin.initialize_app(cred, {
            "storageBucket": _FIREBASE_STORAGE_BUCKET
        })


try:
    _init_firebase()
    _FIREBASE_OK = True
except Exception as _err:
    print(f"[FirebaseUploader] ⚠️  Firebase init failed: {_err}")
    print("                   Uploads will be skipped until credentials are fixed.")
    _FIREBASE_OK = False
# ────────────────────────────────────────────────────────────────────────────


class FirebaseUploader(Node):
    def __init__(self):
        super().__init__('firebase_uploader')

        # ------------------------------------------------------------
        # Cooldown — prevent repeat uploads for the same gap
        # ------------------------------------------------------------
        self._cooldown_sec  = float(os.environ.get("GAP_COOLDOWN_SEC", "5.0"))
        self._last_upload_t = 0.0
        self._upload_lock   = threading.Lock()

        # ------------------------------------------------------------
        # Latest annotated LiDAR frame (updated by /strip/gaps subscriber)
        # ------------------------------------------------------------
        self._latest_lidar_frame: np.ndarray | None = None
        self._lidar_lock = threading.Lock()
        self._bridge = CvBridge()

        # ------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------
        self.lidar_sub = self.create_subscription(
            Image,
            '/strip/gaps',
            self._lidar_cb,
            10
        )

        self.trigger_sub = self.create_subscription(
            String,
            '/gap_events/trigger',
            self._trigger_cb,
            10
        )

        self.get_logger().info(
            "✅ FirebaseUploader ready\n"
            f"   Listening on /gap_events/trigger\n"
            f"   Cooldown: {self._cooldown_sec} s  |  "
            f"Firebase: {'OK' if _FIREBASE_OK else 'NOT CONNECTED'}"
        )

    # ------------------------------------------------------------------
    # Keep a copy of the latest annotated LiDAR frame
    # ------------------------------------------------------------------
    def _lidar_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lidar_lock:
                self._latest_lidar_frame = frame.copy()
        except Exception as e:
            self.get_logger().warn(f"lidar_cb error: {e}")

    # ------------------------------------------------------------------
    # Gap trigger callback — entry point for every detected gap event
    # ------------------------------------------------------------------
    def _trigger_cb(self, msg: String):
        import time

        try:
            payload    = json.loads(msg.data)
            gap_count  = int(payload.get("gap_count",  0))
            distance_m = float(payload.get("distance_m", 0.0))
            scale      = float(payload.get("scale",      1.0))
        except Exception as e:
            self.get_logger().error(f"Bad trigger payload: {e}")
            return

        # Cooldown check
        now = time.monotonic()
        with self._upload_lock:
            if now - self._last_upload_t < self._cooldown_sec:
                self.get_logger().debug("⏳ Upload skipped — still in cooldown window.")
                return
            self._last_upload_t = now

        # Capture Pi CSI camera frame
        camera_frame, tmp_path = self._capture_csi_frame()

        # Snapshot latest LiDAR frame
        with self._lidar_lock:
            lidar_frame = (
                self._latest_lidar_frame.copy()
                if self._latest_lidar_frame is not None
                else None
            )

        # Fire upload in background thread so ROS spin is never blocked
        threading.Thread(
            target=self._upload,
            args=(camera_frame, tmp_path, lidar_frame, gap_count, distance_m, scale),
            daemon=True
        ).start()

    # ------------------------------------------------------------------
    # Capture a frame using rpicam-still (falls back to libcamera-still)
    # ------------------------------------------------------------------
    def _capture_csi_frame(self):
        """
        Returns (frame, tmp_path) where frame is a BGR numpy array and
        tmp_path is the temp file to clean up after upload.
        Returns (None, None) on failure.
        """
        tmp = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
        tmp_path = tmp.name
        tmp.close()

        for cmd in ["rpicam-still", "libcamera-still"]:
            result = subprocess.run(
                [cmd, "-o", tmp_path, "-t", "1000", "--nopreview"],
                capture_output=True,
                text=True
            )
            if result.returncode == 0 and os.path.exists(tmp_path):
                frame = cv2.imread(tmp_path)
                if frame is not None:
                    self.get_logger().info(
                        f"📷 Photo captured with {cmd} "
                        f"({frame.shape[1]}x{frame.shape[0]})"
                    )
                    return frame, tmp_path
                else:
                    self.get_logger().warn(f"📷 {cmd} ran but image unreadable.")
            else:
                self.get_logger().warn(
                    f"📷 {cmd} failed: {result.stderr.strip()}"
                )

        # Both commands failed
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)
        self.get_logger().error(
            "📷 Could not capture image — camera.jpg will be skipped."
        )
        return None, None

    # ------------------------------------------------------------------
    # Firebase upload (runs in background thread)
    # ------------------------------------------------------------------
    def _upload(self, camera_frame: np.ndarray | None,
                tmp_path: str | None,
                lidar_frame: np.ndarray | None,
                gap_count: int,
                distance_m: float,
                scale: float):

        if not _FIREBASE_OK:
            self.get_logger().warn("☁️  Firebase not initialised — skipping upload.")
            return

        try:
            now    = datetime.datetime.utcnow()
            ts_str = now.strftime("%Y%m%d_%H%M%S_") + f"{now.microsecond:06d}"
            folder = f"gap_events/{ts_str}"

            bucket = fb_storage.bucket()
            db     = firestore.client()

            camera_url = None
            lidar_url  = None

            # ── Upload Pi CSI camera image ─────────────────────────────
            if camera_frame is not None and tmp_path is not None:
                with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tf:
                    cv2.imwrite(tf.name, camera_frame,
                                [cv2.IMWRITE_JPEG_QUALITY, 85])
                    blob = bucket.blob(f"{folder}/camera.jpg")
                    blob.upload_from_filename(tf.name, content_type="image/jpeg")
                    blob.make_public()
                    camera_url = blob.public_url
                os.unlink(tf.name)
                os.unlink(tmp_path)   # clean up rpicam-still temp file
            else:
                self.get_logger().warn("📷 No camera frame — camera.jpg skipped.")

            # ── Upload annotated LiDAR strip ───────────────────────────
            if lidar_frame is not None:
                with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tf:
                    cv2.imwrite(tf.name, lidar_frame,
                                [cv2.IMWRITE_JPEG_QUALITY, 85])
                    blob = bucket.blob(f"{folder}/lidar.jpg")
                    blob.upload_from_filename(tf.name, content_type="image/jpeg")
                    blob.make_public()
                    lidar_url = blob.public_url
                os.unlink(tf.name)
            else:
                self.get_logger().warn("🖼️  No LiDAR frame — lidar.jpg skipped.")

            # ── Write Firestore document ───────────────────────────────
            db.collection("gap_events").add({
                "timestamp":  now.isoformat() + "Z",
                "gap_count":  gap_count,
                "distance_m": round(distance_m, 3),
                "scale":      round(scale, 3),
                "camera_url": camera_url,
                "lidar_url":  lidar_url,
            })

            self.get_logger().info(
                f"☁️  Upload complete → {folder}  "
                f"(gaps={gap_count}, dist={distance_m:.2f} m)"
            )

        except Exception as e:
            self.get_logger().error(f"Firebase upload failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseUploader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
