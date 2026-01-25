import serial
import time
import threading
from queue import Queue

class SerialBuffer:
    def __init__(self):
        self.recQueue = Queue()
        self.outQueue = Queue()

    def writeOut(self, msg):
        self.outQueue.put(msg)

    def getAllOut(self):
        outList = []
        while (not self.outQueue.empty()):
            outList.append(self.outQueue.get())
        return outList

    def writeRec(self, msg):
        self.recQueue.put(msg)

    def getAllRec(self):
        recList = []

        while (not self.recQueue.empty()):
            recList.append(self.recQueue.get())

        return recList

def serial_monitor(port, baud, buffer: SerialBuffer, stop_event: threading.Event):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Monitoring Serial Port: {port} @ {baud} baud")

        while not stop_event.is_set():
            outBuf = buffer.getAllOut()
            for out in outBuf:
                out_bytes = str(out + '\n').encode('utf-8')
                print(f"Sent: {str(out)}")
                ser.write(out_bytes)

            line = ser.readline().decode('latin1').rstrip()

            if line:
                timestamp = time.strftime('%H:%M:%S')
                print(f'{timestamp} > {line}')
                buffer.writeRec(line)

            time.sleep(0.01)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Monitoring stopped by user (KeyboardInterrupt).")
    finally:
        if 'ser' in locals() and ser.isOpen():
            ser.close()
            print("Serial port closed.")

def serial_write(buffer: SerialBuffer, stop_event: threading.Event):
    while not stop_event.is_set():
        buffer.writeOut("Hello ESP32!")
        time.sleep(1)


# serial_monitor('COM4', 9600, buf)