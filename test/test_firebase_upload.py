#!/usr/bin/env python3
"""
test_firebase_upload.py
────────────────────────
Standalone test — no ROS required.

1. Opens the USB camera and captures one photo
2. Uploads it to Firebase Storage under  test_uploads/<timestamp>/camera.jpg
3. Writes a Firestore document to the  test_uploads  collection
4. Prints the public URL if successful

Run:
    python3 test_firebase_upload.py

Optional env vars:
    GOOGLE_APPLICATION_CREDENTIALS   path to service-account JSON
    USB_CAM_INDEX                     camera index (default 0)
"""

import os
import sys
import datetime
import tempfile

import cv2
import firebase_admin
from firebase_admin import credentials, firestore, storage as fb_storage

# ── Config ───────────────────────────────────────────────────────────────────
SERVICE_ACCOUNT_PATH = os.environ.get(
    "GOOGLE_APPLICATION_CREDENTIALS",
    os.path.expanduser("~/firebase_service_account.json")
)
STORAGE_BUCKET = "sick-lidar-3.firebasestorage.app"
CAM_INDEX      = int(os.environ.get("USB_CAM_INDEX", "0"))
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


def capture_photo():
    print(f"\n📷 Opening USB camera at index {CAM_INDEX}...")
    cap = cv2.VideoCapture(CAM_INDEX)

    if not cap.isOpened():
        print(f"❌ Could not open camera at index {CAM_INDEX}")
        print("   Try setting USB_CAM_INDEX=1 (or 2) if you have multiple cameras")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)

    # Flush buffer
    for _ in range(2):
        cap.grab()

    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print("❌ Camera read() failed")
        sys.exit(1)

    h, w = frame.shape[:2]
    print(f"✅ Photo captured  ({w}x{h})")
    return frame


def upload(frame):
    now    = datetime.datetime.utcnow()
    ts_str = now.strftime("%Y%m%d_%H%M%S_") + f"{now.microsecond:06d}"
    folder = f"test_uploads/{ts_str}"

    print(f"\n☁️  Uploading to Firebase Storage → {folder}/camera.jpg ...")

    bucket = fb_storage.bucket()
    db     = firestore.client()

    # Save frame to a temp file and upload
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

    # Write Firestore document
    print("\n📝 Writing Firestore document to 'test_uploads' collection...")
    _, doc_ref = db.collection("test_uploads").add({
        "timestamp":  now.isoformat() + "Z",
        "camera_url": camera_url,
        "note":       "manual test upload",
    })
    print(f"✅ Firestore document written  (id: {doc_ref.id})")

    return camera_url


if __name__ == "__main__":
    print("=" * 50)
    print("  Firebase Upload Test")
    print("=" * 50)

    init_firebase()
    frame = capture_photo()
    url   = upload(frame)

    print("\n" + "=" * 50)
    print("✅ All done! Check your Firebase console:")
    print("   Storage  → test_uploads/")
    print("   Firestore → test_uploads collection")
    print(f"\n   Direct image URL:\n   {url}")
    print("=" * 50)
