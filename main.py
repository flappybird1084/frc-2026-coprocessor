import cv2
import socket
import json
import time
import os
from robotpy_apriltag import AprilTagDetector, AprilTagPoseEstimator

# --- CONFIGURATION ---
UDP_IP = "10.10.25.2"  # RoboRIO IP
UDP_PORT = 5800 ##port at which roborio is listening for vision data
MAP_FILE = "field_map.json"
TAG_SIZE = 0.1651  # Meters (6.5 inches)

# Camera Calibration - Replace with your 2026 calibration results!
FOCAL_LENGTH = 650 
CX, CY = 640, 360 

def load_field_map(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            return json.load(f)
    print(f"Error: {filepath} not found!")
    return {}

def main():
    # 1. Load your Tag Data
    field_map = load_field_map(MAP_FILE)
    
    # 2. Setup UDP & Detector
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    detector = AprilTagDetector()
    detector.addFamily("tag36h11")
    
    estimator_config = AprilTagPoseEstimator.Config(
        TAG_SIZE, FOCAL_LENGTH, FOCAL_LENGTH, CX, CY
    )
    estimator = AprilTagPoseEstimator(estimator_config)

    cap = cv2.VideoCapture(0)
    # 2026 Tip: Set high shutter speed/low exposure to stop motion blur
    cap.set(cv2.CAP_PROP_EXPOSURE, 5) 

    print(f"Vision System Online. Tracking {len(field_map)} tags.")

    while True:
        ret, frame = cap.read()
        if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)

        packet = []
        for det in detections:
            tag_id = str(det.getId())
            
            # Only process if it's in our JSON map
            if tag_id in field_map:
                pose = estimator.estimate(det)
                
                # Translation: X (right), Y (down), Z (forward) from camera
                t = pose.translation()
                # Rotation: Rotation of the tag relative to the camera
                r = pose.rotation()

                packet.append({
                    "id": int(tag_id),
                    "x": round(t.x, 3),
                    "y": round(t.y, 3),
                    "z": round(t.z, 3),
                    "rot": [round(r.getQuaternion().w, 3), 
                            round(r.getQuaternion().x, 3), 
                            round(r.getQuaternion().y, 3), 
                            round(r.getQuaternion().z, 3)]
                })

        # 3. Stream to RoboRIO
        if packet:
            message = json.dumps(packet).encode()
            try:
                sock.sendto(message, (UDP_IP, UDP_PORT))
            except Exception as e:
                print(f"Network Error: {e}")

        # Keep CPU usage in check
        time.sleep(0.01)

if __name__ == "__main__":
    main()