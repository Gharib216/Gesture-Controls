import cv2 as cv
import numpy as np
import time
import HandTrackingModule as htm
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
import screen_brightness_control as sbc

# Constants
camera_width, camera_height = 640, 480
cv.namedWindow("Live Cam", cv.WINDOW_NORMAL)
cv.resizeWindow("Live Cam", camera_width, camera_height)

GREEN = (0, 255, 0)
RED = (0, 0, 255)
YELLOW = (0, 255, 255)
vol_color = GREEN
brightness_color = YELLOW
min_dist = 1.5
max_dist = 15.0

# Volume setup
devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))
vol_range = volume.GetVolumeRange()
min_vol, max_vol = vol_range[0], vol_range[1]

# Initialize hand detector
detector = htm.HandDetector(min_detection_confidence=0.7)
cap = cv.VideoCapture(0)
previous_time = 0

if not cap.isOpened():
    print("Error: Could not open the camera.")
else:
    while True:
        success, frame = cap.read()
        if not success:
            break

        hands, frame = detector.findHands(frame, draw=False)

        right_hand = next((h for h in hands if h["type"] == "Right"), None)
        left_hand = next((h for h in hands if h["type"] == "Left"), None)

        vol_percent = int(volume.GetMasterVolumeLevelScalar() * 100)
        brightness_percent = sbc.get_brightness(display=0)[0]

        if right_hand:
            fingers = detector.fingersUp(right_hand)
            if fingers[1] == 1 and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 0:

                lmList = right_hand["lmList"]
                if len(lmList) > 8:
                    p1, p2, p3 = lmList[4], lmList[8], lmList[5]

                    raw_dist = detector.getDistance(p1, p2, frame, vol_color)[0]
                    ref_len = max(1e-5, detector.getDistance(p2, p3)[0])
                    estimated_dist = (raw_dist / ref_len) * 8.5
                    clamped_dist = max(min_dist, min(max_dist, estimated_dist))
                    vol_scalar = np.interp(clamped_dist, [min_dist, max_dist], [0.0, 1.0])
                    volume.SetMasterVolumeLevelScalar(vol_scalar, None)
                    vol_percent = int(vol_scalar * 100)

            vol_color = RED if vol_percent > 85 else GREEN
            cv.putText(frame, 'Vol', (580, 80), cv.FONT_HERSHEY_PLAIN, 1.6, GREEN, 3)
            cv.rectangle(frame, (580, 88), (620, 390), GREEN, 3)
            cv.rectangle(frame, (583, 390 - 3 * vol_percent), (617, 388), vol_color, -1)
            cv.putText(frame, f'{vol_percent}%', (575, 415), cv.FONT_HERSHEY_PLAIN, 1.4, GREEN, 2)

        if left_hand:
            fingers = detector.fingersUp(left_hand)
            if fingers[1] == 1 and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 0:

                lmList = left_hand["lmList"]
                if len(lmList) > 8:
                    p1, p2, p3 = lmList[4], lmList[8], lmList[5]
                    raw_dist = detector.getDistance(p1, p2, frame, brightness_color)[0]
                    ref_len = max(1e-5, detector.getDistance(p2, p3)[0])
                    estimated_dist = (raw_dist / ref_len) * 8.5
                    clamped_dist = max(min_dist, min(max_dist, estimated_dist))

                    brightness_percent = int(np.interp(clamped_dist, [min_dist, max_dist], [0, 100]))
                    sbc.set_brightness(brightness_percent)

            brightness_color = RED if brightness_percent > 85 else YELLOW
            cv.putText(frame, 'brightness', (20, 80), cv.FONT_HERSHEY_PLAIN, 1.6, YELLOW, 3)
            cv.rectangle(frame, (20, 88), (60, 390), YELLOW, 3)
            cv.rectangle(frame, (23, 390 - 3 * brightness_percent), (57, 388), brightness_color, -1)
            cv.putText(frame, f'{brightness_percent}%', (15, 415), cv.FONT_HERSHEY_PLAIN, 1.4, YELLOW, 2)

        current_time = time.time()
        fps = 1 / (current_time - previous_time)
        previous_time = current_time
        cv.putText(frame, f'FPS: {int(fps)}', (10, 50), cv.FONT_HERSHEY_PLAIN, 3, GREEN, 3)

        cv.imshow('Live Cam', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv.destroyAllWindows()
