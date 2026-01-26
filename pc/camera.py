import cv2
from cvzone.HandTrackingModule import HandDetector
import threading
import math
import time

from serial_monitor import SerialBuffer
from queue import Queue

SCREEN_HEIGHT = 500
SCREEN_WIDTH = 700

HEIGHT_CONV = 25
WIDTH_CONV = 23

FPS = 30

fourcc = cv2.VideoWriter_fourcc(*"MJPG")

video_writer = None
recording_path = None
recording_active = False
release_flag = False

video_writer = None

def coordinate_transform(x, y):
    #(700 - 0, 500 - 0) -> (700 - 0, 0 - 500)
    x, y = x, -(y - SCREEN_HEIGHT/2) + SCREEN_HEIGHT/2

    x, y = x/WIDTH_CONV - 15, y/HEIGHT_CONV
    return (x, y)


L1 = 10.0
L2 = 7

L1_PX = L1*24
L2_PX = L2*24

def inverse_kinematics(x, y):
    r2 = x * x + y * y

    # Within semicircle
    if r2 > (L1 + L2) ** 2 or r2 < (L1 - L2) ** 2:
        return (0, 0)

    # Elbow angle
    c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2)) 
    # Choose elbow configuration: + for one, - for the other
    s2 = -math.sqrt(1.0 - c2 * c2)
    theta2 = math.atan2(s2, c2)   # use -s2 for the other branch

    # Shoulder angle
    phi = math.atan2(y, x)
    k1 = L1 + L2 * c2
    k2 = L2 * s2
    theta1 = phi - math.atan2(k2, k1)

    return theta1, theta2


def arm_points_from_angles(theta1, theta2):
    bx, by = int(SCREEN_WIDTH/2), int(SCREEN_HEIGHT)

    # First joint (shoulder) is at base
    x1 = bx + int(L1_PX * math.cos(theta1))
    y1 = by - int(L1_PX * math.sin(theta1))  # minus because image y goes down

    # End effector
    theta12 = theta1 + theta2
    x2 = x1 + int(L2_PX * math.cos(theta12))
    y2 = y1 - int(L2_PX * math.sin(theta12))

    return (bx, by), (x1, y1), (x2, y2)

def run_camera(frame_buffer: Queue,
               buffer: SerialBuffer,
               joint_buffer: Queue,
               stability_buffer: Queue,
               stop_event: threading.Event):
    global video_writer, release_flag
    print("Camera thread started")
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    detector = HandDetector(maxHands=2, detectionCon=0.4)

    target_period = 1.0 / FPS
    last_time = time.time()

    while not stop_event.is_set():
        success, img = cap.read()
        if not success or img is None:
            print("Failed to read from camera")
            break

        if recording_active:
            print("Recording active")
            h, w = img.shape[:2]
            print(f"Creating video writer with size {w}x{h}")
            video_writer = cv2.VideoWriter(recording_path, fourcc, FPS, (w, h))
            print(f"Video writer created: {video_writer.isOpened()}")

        hands, img = detector.findHands(img, draw=True, flipType=True)

        if hands:
            hand = hands[0]
            lmList = hand["lmList"]

            fingers = detector.fingersUp(hand)
            is_pointing = (fingers == [0, 1, 0, 0, 0])

            # --- STABILITY LOGGING ---
            # log time + fingertip position for stability calculations
            now = time.time()
            if stability_buffer.full():
                stability_buffer.get_nowait()
            stability_buffer.put((now, lmList[13][0], lmList[13][1]))
            # -------------------------

            if False:
                cv2.imshow("Hands", img)
                if frame_buffer.full():
                    frame_buffer.get_nowait()
                frame_buffer.put(cv2.flip(img, -1))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            else:
                x_px, y_px, z = lmList[8]
                cv2.circle(img, (x_px, y_px), 8, (0, 0, 255), -1)

                # transform to your working coordinates
                x, y = coordinate_transform(x_px, y_px)

                theta1, theta2 = inverse_kinematics(x, y)
                if theta1 == 0 and theta2 == 0:
                    # unreachable pose; still keep stability log, just skip IK
                    pass
                else:
                    p0, p1, p2 = arm_points_from_angles(theta1, theta2)

                    cv2.arrowedLine(img, p0, p1, (255, 0, 0), 4, tipLength=0.1)
                    cv2.arrowedLine(img, p1, p2, (0, 255, 0), 4, tipLength=0.1)

                    theta1 = math.degrees(theta1)
                    theta2 = math.degrees(theta2)
                    theta1 = -(theta1 - 90) + 90
                    theta2 = -theta2

                    buffer.writeOut(f"{theta1:.2f} {theta2:.2f}")

                    if joint_buffer.full():
                        joint_buffer.get_nowait()
                    joint_buffer.put((theta1, theta2))

        if video_writer:
            print("Writing frame")
            video_writer.write(img)

        cv2.imshow("Hands", img)

        if frame_buffer.full():
            frame_buffer.get_nowait()
        frame_buffer.put(cv2.flip(img, -1))

        now_time = time.time()
        dt = now_time - last_time
        if dt < target_period:
            time.sleep(target_period - dt)
        last_time = time.time()

        if release_flag and video_writer:
            print("Releasing video writer")
            video_writer.release()
            video_writer = None
            release_flag = False
            print("Released")

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
