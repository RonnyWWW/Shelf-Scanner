#!/usr/bin/env python3
import datetime
import json
import os
import subprocess
import tempfile
import threading
import time

import cv2
import firebase_admin
import numpy as np
import rclpy
from cv_bridge import CvBridge
from firebase_admin import credentials, firestore, storage as fb_storage
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

_SERVICE_ACCOUNT_PATH = os.environ.get('GOOGLE_APPLICATION_CREDENTIALS', os.path.expanduser('~/firebase_service_account.json'))
_FIREBASE_STORAGE_BUCKET = 'sick-lidar-3.firebasestorage.app'


def _init_firebase():
    if not firebase_admin._apps:
        cred = credentials.Certificate(_SERVICE_ACCOUNT_PATH)
        firebase_admin.initialize_app(cred, {'storageBucket': _FIREBASE_STORAGE_BUCKET})


try:
    _init_firebase()
    _FIREBASE_OK = True
except Exception as _err:
    print(f'[FirebaseUploader] Firebase init failed: {_err}')
    _FIREBASE_OK = False


class FirebaseUploader(Node):
    def __init__(self):
        super().__init__('firebase_uploader')
        self._per_gap_cooldown_sec = float(os.environ.get('GAP_COOLDOWN_SEC', '8.0'))
        self._global_cooldown_sec = float(os.environ.get('GAP_GLOBAL_COOLDOWN_SEC', '0.5'))
        self._last_global_upload_t = 0.0
        self._last_gap_upload_t = {}
        self._upload_lock = threading.Lock()

        self._latest_lidar_frame: np.ndarray | None = None
        self._lidar_lock = threading.Lock()
        self._bridge = CvBridge()

        self.create_subscription(Image, '/strip/gaps', self._lidar_cb, 10)
        self.create_subscription(String, '/gap_events/trigger', self._trigger_cb, 10)

        self.get_logger().info(
            f'✅ FirebaseUploader v2 ready | per-gap cooldown={self._per_gap_cooldown_sec}s | global cooldown={self._global_cooldown_sec}s'
        )

    def _lidar_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lidar_lock:
                self._latest_lidar_frame = frame.copy()
        except Exception as e:
            self.get_logger().warn(f'lidar_cb error: {e}')

    def _trigger_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Bad trigger payload: {e}')
            return

        gap_id = int(payload.get('gap_id', -1))
        now = time.monotonic()
        with self._upload_lock:
            if now - self._last_global_upload_t < self._global_cooldown_sec:
                return
            if gap_id >= 0 and now - self._last_gap_upload_t.get(gap_id, -1e9) < self._per_gap_cooldown_sec:
                return
            self._last_global_upload_t = now
            if gap_id >= 0:
                self._last_gap_upload_t[gap_id] = now

        camera_frame, tmp_path = self._capture_csi_frame()
        with self._lidar_lock:
            lidar_frame = self._latest_lidar_frame.copy() if self._latest_lidar_frame is not None else None

        threading.Thread(target=self._upload, args=(camera_frame, tmp_path, lidar_frame, payload), daemon=True).start()

    def _capture_csi_frame(self):
        tmp = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
        tmp_path = tmp.name
        tmp.close()
        for cmd in ['rpicam-still', 'libcamera-still']:
            result = subprocess.run([cmd, '-o', tmp_path, '-t', '1000', '--nopreview'], capture_output=True, text=True)
            if result.returncode == 0 and os.path.exists(tmp_path):
                frame = cv2.imread(tmp_path)
                if frame is not None:
                    return frame, tmp_path
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)
        return None, None

    def _upload(self, camera_frame, tmp_path, lidar_frame, payload):
        if not _FIREBASE_OK:
            self.get_logger().warn('Firebase not initialised — skipping upload.')
            return
        try:
            now = datetime.datetime.utcnow()
            ts_str = now.strftime('%Y%m%d_%H%M%S_') + f'{now.microsecond:06d}'
            folder = f'gap_events/{ts_str}'
            bucket = fb_storage.bucket()
            db = firestore.client()
            camera_url = None
            lidar_url = None

            if camera_frame is not None and tmp_path is not None:
                with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tf:
                    cv2.imwrite(tf.name, camera_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    blob = bucket.blob(f'{folder}/camera.jpg')
                    blob.upload_from_filename(tf.name, content_type='image/jpeg')
                    blob.make_public()
                    camera_url = blob.public_url
                os.unlink(tf.name)
                os.unlink(tmp_path)

            if lidar_frame is not None:
                with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tf:
                    cv2.imwrite(tf.name, lidar_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    blob = bucket.blob(f'{folder}/lidar.jpg')
                    blob.upload_from_filename(tf.name, content_type='image/jpeg')
                    blob.make_public()
                    lidar_url = blob.public_url
                os.unlink(tf.name)

            doc = {
                'timestamp': now.isoformat() + 'Z',
                'event_type': payload.get('event_type', 'new_gap'),
                'gap_id': int(payload.get('gap_id', -1)),
                'gap_count': int(payload.get('gap_count', 1)),
                'distance_m': float(payload.get('distance_m', 0.0)),
                'shelf_idx': int(payload.get('shelf_idx', -1)),
                'x_center': int(payload.get('x_center', -1)),
                'y_center': int(payload.get('y_center', -1)),
                'width_px': int(payload.get('width_px', -1)),
                'height_px': int(payload.get('height_px', -1)),
                'camera_url': camera_url,
                'lidar_url': lidar_url,
            }
            db.collection('gap_events').add(doc)
            self.get_logger().info(f"Upload complete → gap_id={doc['gap_id']} shelf={doc['shelf_idx']}")
        except Exception as e:
            self.get_logger().error(f'Firebase upload failed: {e}')


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
