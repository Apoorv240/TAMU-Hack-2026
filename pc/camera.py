import cv2
from cvzone.HandTrackingModule import HandDetector
import threading
import math

from serial_monitor import SerialBuffer
from queue import Queue

SCREEN_HEIGHT = 500
SCREEN_WIDTH = 700

HEIGHT_CONV = 25
WIDTH_CONV = 23

def coordinate_transform(x, y):
    #(700 - 0, 500 - 0) -> (700 - 0, 0 - 500)
    x, y = x, -(y - SCREEN_HEIGHT/2) + SCREEN_HEIGHT/2

    x, y = x/WIDTH_CONV - 15, y/HEIGHT_CONV
    return (x, y)


L1 = 10.0
L2 = 10.0

L1_PX = L1*24
L2_PX = L1*24

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

def run_camera(frame_buffer: Queue, buffer: SerialBuffer, stop_event: threading.Event):
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # try 0, if that fails try 1
    detector = HandDetector(maxHands=2, detectionCon=0.4)

    while not stop_event.is_set():
        success, img = cap.read()
        if not success or img is None:
            print("Failed to read from camera")
            break

        # now it's safe to call findHands
        hands, img = detector.findHands(img, draw=True, flipType=True)

        if hands:
            hand = hands[0]
            lmList = hand["lmList"]
            x, y, z = lmList[8]
            cv2.circle(img, (x, y), 8, (0, 0, 255), -1)

            x, y = coordinate_transform(x, y)

            # print(f"({x}, {y})")

            theta1, theta2 = inverse_kinematics(x, y)

            if theta1 == 0 and theta2 == 0:
                continue

            p0, p1, p2 = arm_points_from_angles(theta1, theta2)

            # Draw link 1
            cv2.arrowedLine(img, p0, p1, (255, 0, 0), 4, tipLength=0.1)

            # Draw link 2
            cv2.arrowedLine(img, p1, p2, (0, 255, 0), 4, tipLength=0.1)

            buffer.writeOut(f"{theta1:.2f} {theta2:.2f}")

        cv2.imshow("Hands", cv2.flip(img, -1))

        if frame_buffer.full():
            frame_buffer.get_nowait()
        frame_buffer.put(cv2.flip(img, -1))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
