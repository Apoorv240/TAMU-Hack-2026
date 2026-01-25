from camera import run_camera
from serial_monitor import serial_monitor, serial_write, SerialBuffer
import time
import threading

stop_event = threading.Event()

buf = SerialBuffer()

t1 = threading.Thread(target=serial_monitor, args=('COM4', 9600, buf, stop_event,), daemon=True)
# t2 = threading.Thread(target=serial_write, args=(buf, stop_event,), daemon=True)
t3 = threading.Thread(target=run_camera, args=(buf, stop_event,), daemon=True)

t1.start()
# t2.start()
t3.start()

try:
    while True:
        time.sleep(0.2)
except KeyboardInterrupt:
    print("Ctrl-C pressed, stopping...")
    stop_event.set()
    t1.join(timeout=1.0)
    # t2.join(timeout=1.0)
    t3.join(timeout=1.0)
    print("Exited cleanly.")