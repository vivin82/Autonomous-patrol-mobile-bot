#!/usr/bin/env python3
import rospy
import cv2
import json
import os
import time
import requests
import numpy as np
from datetime import datetime
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

# ==========================================
# TELEGRAM BOT SETTINGS
# ==========================================
BOT_TOKEN = '8720927123:AAHfwrIHOZbt4gCTqAE8B15IrxP8l3CNfnE'
CHAT_ID = '8693155973'
COOLDOWN_SECONDS = 10              
LAST_ALERT_TIME = 0                

# ==========================================
# CAMERA FIX: 
# Since ROS is grabbing your Laptop Camera, 
# OpenCV needs to grab your USB Camera.
# On Linux, USB cams are usually index 2 or 1.
# ==========================================
OPENCV_CAM_INDEX = 2 
opencv_camera = cv2.VideoCapture(OPENCV_CAM_INDEX)

LOG_FILE = os.path.expanduser("~/patrol_ws/detections.log")
model  = YOLO("yolov8n.pt")
bridge = CvBridge()

def send_telegram(image_frame, count):
    """Saves the stitched frame and sends it via Telegram."""
    temp_path = '/tmp/alert.jpg'
    cv2.imwrite(temp_path, image_frame)
    url = f'https://api.telegram.org/bot{BOT_TOKEN}/sendPhoto'
    data = {'chat_id': CHAT_ID, 'caption': f'🚨 DUAL ALERT: {count} Intruder(s) Detected!'}
    
    try:
        with open(temp_path, 'rb') as photo:
            response = requests.post(url, data=data, files={'photo': photo}, timeout=15)
            if response.status_code == 200:
                rospy.loginfo("Telegram dual-photo sent successfully!")
            else:
                rospy.logwarn(f"TELEGRAM ERROR SAYS: {response.text}")
    except Exception as e:
        rospy.logwarn(f"Network error: {e}")

def detect(img_msg, pub):
    global LAST_ALERT_TIME
    
    # 1. BACK VIEW: Get the ROS Topic (Which we now know is the Laptop Cam)
    ros_frame = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    ros_frame = cv2.resize(ros_frame, (640, 480)) # Force size
    cv2.putText(ros_frame, "BACK VIEW (LAPTOP)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    
    # 2. FRONT VIEW: Get the OpenCV feed (Which should be the USB Cam)
    ret, cv_cam_frame = opencv_camera.read()
    if not ret:
        # If it still fails, draw a red error box
        cv_cam_frame = np.zeros_like(ros_frame)
        cv2.putText(cv_cam_frame, "NO USB FEED", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        cv2.putText(cv_cam_frame, f"Tried Index: {OPENCV_CAM_INDEX}", (200, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
    else:
        cv_cam_frame = cv2.resize(cv_cam_frame, (640, 480)) # Force size
        cv2.putText(cv_cam_frame, "FRONT VIEW (USB)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # 3. Stitch them together (Left = Front/USB, Right = Back/Laptop)
    combined_frame = np.concatenate((cv_cam_frame, ros_frame), axis=1)

    # 4. Run YOLO ONCE on the ultra-wide image
    results = model(combined_frame, classes=[0], conf=0.5, verbose=False)

    for r in results:
        if len(r.boxes) > 0:
            count = len(r.boxes)
            entry = {
                "timestamp": datetime.now().isoformat(),
                "persons":   count,
                "conf":      [round(float(b.conf), 2) for b in r.boxes]
            }
            msg_str = json.dumps(entry)
            pub.publish(msg_str)

            # Telegram Trigger
            current_time = time.time()
            if (current_time - LAST_ALERT_TIME) > COOLDOWN_SECONDS:
                rospy.logwarn(f"DUAL ALERT: {count} person(s) detected! Sending to Telegram...")
                with open(LOG_FILE, "a") as f:
                    f.write(msg_str + "\n")
                
                annotated_frame = r.plot() 
                send_telegram(annotated_frame, count)
                LAST_ALERT_TIME = current_time

def cleanup():
    """Releases the camera when you stop the ROS node"""
    opencv_camera.release()
    rospy.loginfo("Camera released.")

def main():
    rospy.init_node("person_detector")
    rospy.on_shutdown(cleanup)
    
    pub = rospy.Publisher("/patrol/detections", String, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw", Image, lambda msg: detect(msg, pub), queue_size=1, buff_size=2**24)
    
    rospy.loginfo("Dual-Camera Person detector running with Telegram alerts.")
    rospy.spin()

if __name__ == "__main__":
    main()
