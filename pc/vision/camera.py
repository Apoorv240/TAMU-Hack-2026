import cv2
from cvzone.HandTrackingModule import HandDetector
import threading
import math
import time

from ..arm.kinematics import inverse_kinematics, L1_PX, L2_PX

from ..util.serial_monitor import SerialBuffer
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
               stability_buffer: Queue,
               target_buffer: Queue,
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

                if target_buffer.full():
                    target_buffer.get_nowait()
                target_buffer.put((x, y))

                theta1, theta2 = inverse_kinematics(x, y)
                if theta1 == 0 and theta2 == 0:
                    # unreachable pose; still keep stability log, just skip IK
                    pass
                else:
                    p0, p1, p2 = arm_points_from_angles(theta1, theta2)

                    cv2.arrowedLine(img, p0, p1, (255, 0, 0), 4, tipLength=0.1)
                    cv2.arrowedLine(img, p1, p2, (0, 255, 0), 4, tipLength=0.1)

                    buffer.writeOut(f"{theta1:.2f} {theta2:.2f}")

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
