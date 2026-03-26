#!/usr/bin/env python3
"""
test_firebase_upload.py
────────────────────────
Standalone Firebase upload test — no ROS, no camera required.

Generates a dummy test image and uploads it to Firebase Storage,
then writes a Firestore document to confirm the full pipeline works.

Run:
    python3 test_firebase_upload.py

Optional env vars:
    GOOGLE_APPLICATION_CREDENTIALS   path to service-account JSON
"""

import os
import sys
import datetime
import tempfile

import cv2
import numpy as np
import firebase_admin
from firebase_admin import credentials, firestore, storage as fb_storage

# ── Config ───────────────────────────────────────────────────────────────────
SERVICE_ACCOUNT_PATH = os.environ.get(
    "GOOGLE_APPLICATION_CREDENTIALS",
    os.path.expanduser("~/firebase_service_account.json")
)
STORAGE_BUCKET = "sick-lidar-3.firebasestorage.app"
# ─────────────────────────────────────────────────────────────────────────────


def init_firebase():
    print(f"🔑 Loading credentials from: {SERVICE_ACCOUNT_PATH}")
    if not os.path.exists(SERVICE_ACCOUNT_PATH):
        print("❌ Service account file not found.")
        print("   Download it from Firebase Console → Project Settings")
        print("   → Service Accounts → Generate new private key")
        print(f"   Then save it to: {SERVICE_ACCOUNT_PATH}")
        sys.exit(1)

    cred = credentials.Certificate(SERVICE_ACCOUNT_PATH)
    firebase_admin.initialize_app(cred, {"storageBucket": STORAGE_BUCKET})
    print("✅ Firebase initialised")


def make_dummy_image():
    """Generate a recognisable test image — grey background with text."""
    img = np.ones((720, 1280, 3), dtype=np.uint8) * 60  # dark grey

    # Coloured banner across the top
    img[:80, :] = (30, 120, 200)   # orange-ish in BGR

    cv2.putText(img, "FIREBASE UPLOAD TEST",
                (320, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

    now_str = datetime.datetime.utcnow().strftime("%Y-%m-%d  %H:%M:%S UTC")
    cv2.putText(img, now_str,
                (430, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2)

    cv2.putText(img, "Shelf Scanner  —  Raspberry Pi 4",
                (390, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 230, 180), 2)

    cv2.putText(img, "Camera: DUMMY IMAGE (no physical camera)",
                (300, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 180, 255), 2)

    # Draw a simple shelf-gap illustration in the centre
    shelf_y = [340, 420, 500, 580]
    for sy in shelf_y:
        cv2.rectangle(img, (200, sy), (1080, sy + 15), (80, 80, 160), -1)

    # Gap box
    cv2.rectangle(img, (550, 355), (750, 415), (0, 0, 255), 3)
    cv2.putText(img, "GAP", (615, 395),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    print("✅ Dummy test image generated  (1280x720)")
    return img


def upload(frame):
    now    = datetime.datetime.utcnow()
    ts_str = now.strftime("%Y%m%d_%H%M%S_") + f"{now.microsecond:06d}"
    folder = f"test_uploads/{ts_str}"

    print(f"\n☁️  Uploading to Firebase Storage → {folder}/camera.jpg ...")

    bucket = fb_storage.bucket()
    db     = firestore.client()

    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tf:
        cv2.imwrite(tf.name, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        tmp_path = tf.name

    blob = bucket.blob(f"{folder}/camera.jpg")
    blob.upload_from_filename(tmp_path, content_type="image/jpeg")
    blob.make_public()
    os.unlink(tmp_path)

    camera_url = blob.public_url
    print(f"✅ Image uploaded")
    print(f"   URL: {camera_url}")

    print("\n📝 Writing Firestore document to 'test_uploads' collection...")
    _, doc_ref = db.collection("test_uploads").add({
        "timestamp":  now.isoformat() + "Z",
        "camera_url": camera_url,
        "note":       "dummy image test — no physical camera",
    })
    print(f"✅ Firestore document written  (id: {doc_ref.id})")

    return camera_url


if __name__ == "__main__":
    print("=" * 55)
    print("  Firebase Upload Test  (dummy image)")
    print("=" * 55)

    init_firebase()
    frame = make_dummy_image()
    url   = upload(frame)

    print("\n" + "=" * 55)
    print("✅ All done! Check your Firebase console:")
    print("   Storage   → test_uploads/")
    print("   Firestore → test_uploads collection")
    print(f"\n   Direct image URL:\n   {url}")
    print("=" * 55)
