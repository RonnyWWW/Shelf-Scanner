#!/usr/bin/env python3
"""
test_firebase_upload.py
────────────────────────
Standalone test — no ROS required.

1. Captures a photo using rpicam-still (Pi CSI camera)
2. Uploads it to Firebase Storage under  test_uploads/<timestamp>/camera.jpg
3. Writes a Firestore document to the  test_uploads  collection
4. Prints the public URL if successful

Run:
    python3 test_firebase_upload.py

Optional env vars:
    GOOGLE_APPLICATION_CREDENTIALS   path to service-account JSON
"""

import os
import sys
import datetime
import tempfile
import subprocess

import cv2
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


def capture_photo():
    """Capture using rpicam-still, fall back to libcamera-still."""
    tmp = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
    tmp_path = tmp.name
    tmp.close()

    # Try rpicam-still first, then libcamera-still
    for cmd in ["rpicam-still", "libcamera-still"]:
        print(f"\n📷 Trying {cmd}...")
        result = subprocess.run(
            [cmd, "-o", tmp_path, "-t", "1", "--nopreview"],
            capture_output=True,
            text=True
        )
        if result.returncode == 0 and os.path.exists(tmp_path):
            frame = cv2.imread(tmp_path)
            if frame is not None:
                h, w = frame.shape[:2]
                print(f"✅ Photo captured with {cmd}  ({w}x{h})")
                return frame, tmp_path
            else:
                print(f"   ⚠️  {cmd} ran but image could not be read")
        else:
            print(f"   ⚠️  {cmd} failed: {result.stderr.strip()}")

    os.unlink(tmp_path)
    print("❌ Could not capture image with rpicam-still or libcamera-still")
    print("   Make sure the camera is connected and the commands are in PATH")
    print("   Check with: which rpicam-still")
    sys.exit(1)


def upload(frame, tmp_path):
    now    = datetime.datetime.utcnow()
    ts_str = now.strftime("%Y%m%d_%H%M%S_") + f"{now.microsecond:06d}"
    folder = f"test_uploads/{ts_str}"

    print(f"\n☁️  Uploading to Firebase Storage → {folder}/camera.jpg ...")

    bucket = fb_storage.bucket()
    db     = firestore.client()

    # Re-encode at 85% quality before uploading
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tf:
        cv2.imwrite(tf.name, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        upload_path = tf.name

    blob = bucket.blob(f"{folder}/camera.jpg")
    blob.upload_from_filename(upload_path, content_type="image/jpeg")
    blob.make_public()
    camera_url = blob.public_url

    os.unlink(upload_path)
    os.unlink(tmp_path)

    print(f"✅ Image uploaded")
    print(f"   URL: {camera_url}")

    print("\n📝 Writing Firestore document to 'test_uploads' collection...")
    _, doc_ref = db.collection("test_uploads").add({
        "timestamp":  now.isoformat() + "Z",
        "camera_url": camera_url,
        "note":       "manual test upload — rpicam-still",
    })
    print(f"✅ Firestore document written  (id: {doc_ref.id})")

    return camera_url


if __name__ == "__main__":
    print("=" * 50)
    print("  Firebase Upload Test (rpicam-still)")
    print("=" * 50)

    init_firebase()
    frame, tmp_path = capture_photo()
    url = upload(frame, tmp_path)

    print("\n" + "=" * 50)
    print("✅ All done! Check your Firebase console:")
    print("   Storage  → test_uploads/")
    print("   Firestore → test_uploads collection")
    print(f"\n   Direct image URL:\n   {url}")
    print("=" * 50)
