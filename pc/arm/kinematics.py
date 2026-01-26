import math
from queue import Queue
import time

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

def process_joints(joint_buffer: Queue, pid_buffer: Queue, stop_event):
    cur1 = 0.0
    cur2 = 0.0

    # Max change per cycle (units per 10 ms)
    max_step = 3   # tune for speed vs smoothness
    dt = 0.01         # 10 ms

    target1 = cur1
    target2 = cur2

    while not stop_event.is_set():
        # Grab the latest target if available (drop older ones)
        while not joint_buffer.empty():
            target1, target2 = joint_buffer.get()

        # Compute increments toward target
        diff1 = target1 - cur1
        diff2 = target2 - cur2

        # Clamp step size for each joint
        if abs(diff1) > max_step:
            cur1 += max_step if diff1 > 0 else -max_step
        else:
            cur1 = target1

        if abs(diff2) > max_step:
            cur2 += max_step if diff2 > 0 else -max_step
        else:
            cur2 = target2

        # Emit the new commanded positions (latest only)
        if pid_buffer.full():
            pid_buffer.get_nowait()
        pid_buffer.put((cur1, cur2))

        time.sleep(dt)