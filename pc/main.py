from pc.vision.camera import run_camera, FPS
from pc.util.serial_monitor import serial_monitor, SerialBuffer
import time
import threading
from queue import Queue

import requests
import os

from pc.arm.kinematics import process_joints
from pc.vision.stability import stability_worker
from pc.site.site import video_writer, socketio, app
import pc.site as site

# Shared State
stop_event = threading.Event()
serial_buf = SerialBuffer()
frame_buf = Queue(maxsize=1)
joint_buf = Queue(maxsize=1)
stability_buf = Queue(maxsize=1000)
pid_buf = Queue(maxsize=1)

site.site.frame_buf = frame_buf
site.site.stop_event = stop_event


def write_esp32(joint_buf: Queue, stop_event):
    prev_t1 = 0
    prev_t2 = 0
    while not stop_event.is_set():
        t1, t2 = joint_buf.get()
        try:
            if (t1 - prev_t1 > 20 or t2 - prev_t2 > 10):
                requests.get(f"http://192.168.4.1/arm?j1={int(t1)}&j2={int(t2)}", timeout=0.1)
        except requests.RequestException:
            pass
        time.sleep(0.03)


def disable_esp32():
    try:
        requests.get("http://192.168.4.1/arm?enable=0", timeout=0.1)
    except requests.RequestException:
        pass


def main():
    t1 = threading.Thread(
        target=serial_monitor,
        args=('COM4', 9600, serial_buf, stop_event),
        daemon=True,
    )
    t3 = threading.Thread(
        target=run_camera,
        args=(frame_buf, serial_buf, joint_buf, stability_buf, stop_event),
        daemon=True,
    )
    t4 = threading.Thread(
        target=write_esp32,
        args=(pid_buf, stop_event,),
        daemon=True
    )
    t5 = threading.Thread(
        target=stability_worker,
        args=(stability_buf, socketio, stop_event),
        daemon=True
    )
    t6 = threading.Thread(
        target=process_joints,
        args=(joint_buf, pid_buf, stop_event),
        daemon=True
    )

    t1.start()
    t3.start()
    t4.start()
    t5.start()
    t6.start()

    try:
        socketio.run(app, host="0.0.0.0", port=5000)
    except KeyboardInterrupt:
        print("Ctrl-C pressed, stopping...")
    finally:
        stop_event.set()
        t1.join(timeout=1.0)
        t3.join(timeout=1.0)
        t4.join(timeout=1.0)
        t5.join(timeout=1.0)
        t6.join(timeout=1.0)
        if video_writer is not None:
            video_writer.release()
        print("Exited cleanly.")


if __name__ == "__main__":
    main()
