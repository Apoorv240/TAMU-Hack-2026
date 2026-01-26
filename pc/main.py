from camera import run_camera, FPS
from serial_monitor import serial_monitor, SerialBuffer
import time
import threading
from queue import Queue

import requests

from flask import Flask, Response, send_from_directory, render_template, jsonify
from flask_socketio import SocketIO, emit
import os
import cv2
from glob import glob
import math
import statistics as st

# Shared State
stop_event = threading.Event()
serial_buf = SerialBuffer()
frame_buf = Queue(maxsize=1)
joint_buf = Queue(maxsize=1)
stability_buf = Queue(maxsize=1000)

app = Flask(__name__, static_folder='recordings')
socketio = SocketIO(app)

os.makedirs("recordings", exist_ok=True)
fourcc = cv2.VideoWriter_fourcc(*'avc1') 

video_writer = None
current_case_id = None
recording_active = False


def list_recordings():
    files = sorted(
        glob(os.path.join("recordings", "*.mp4")),
        key=os.path.getmtime,
        reverse=True,
    )
    # return just filenames
    return [os.path.basename(f) for f in files]


def gen_stream():
    global video_writer
    while not stop_event.is_set():
        frame = frame_buf.get()  # blocks

        if frame is None:
            continue

        # init writer when first frame arrives and recording is active
        if recording_active:
            if video_writer is None:
                h, w, _ = frame.shape
                filename = f"{current_case_id}.mp4" if current_case_id else "latest.mp4"
                path = os.path.join("recordings", filename)
                video_writer = cv2.VideoWriter(path, fourcc, FPS, (w, h))

            video_writer.write(frame)
        else:
            # if we stop recording, close writer
            if video_writer is not None:
                video_writer.release()
                video_writer = None

        # live stream (always)
        ret, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        if not ret:
            continue
        frame_bytes = jpeg.tobytes()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")


@app.route("/")
def index():
    recordings = list_recordings()
    return render_template("index.html", recordings=recordings)


@app.route("/video")
def video():
    return Response(gen_stream(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/recordings")
def recordings_list():
    return jsonify(list_recordings())


@app.route("/download/<name>")
def download(name):
    return send_from_directory("recordings", name, as_attachment=True, mimetype="video/mp4")


@app.route("/play/<name>")
def play_recording(name):
    # simple static file; the front-end can use this as a video src
    return send_from_directory("recordings", name)


@app.route("/start_case", methods=["POST"])
def start_case():
    global current_case_id, recording_active
    current_case_id = time.strftime("CASE-%Y%m%d-%H%M%S")
    recording_active = True
    return jsonify({"case_id": current_case_id})


@app.route("/stop_case", methods=["POST"])
def stop_case():
    global recording_active
    recording_active = False
    return jsonify({"ok": True})


def write_esp32(joint_buf: Queue, stop_event):
    while not stop_event.is_set():
        t1, t2 = joint_buf.get()
        try:
            requests.get(f"http://192.168.4.1/arm?j1={int(t1)}&j2={int(t2)}", timeout=0.1)
        except requests.RequestException:
            pass
        time.sleep(0.03)


def smooth_series(xs, alpha=0.6):
    """Simple exponential smoothing to reduce noise."""
    out = []
    prev = None
    for x in xs:
        if prev is None:
            prev = x
        else:
            prev = alpha * x + (1 - alpha) * prev
        out.append(prev)
    return out


def compute_stability(log):
    # log: list of (t, x, y)
    if len(log) < 5:
        return None

    ts, xs, ys = zip(*log)

    # 1) Stronger smoothing to avoid amplifying pixel jitter
    xs = smooth_series(xs, alpha=0.6)
    ys = smooth_series(ys, alpha=0.6)

    # 2) Use a step > 1 for derivatives to increase dt and reduce noise
    step = 2  # or 3 if you want even more smoothing
    path = 0.0
    velocities = []
    accelerations = []
    jerks = []

    # velocities from smoothed positions
    for i in range(0, len(ts) - step, step):
        dt = ts[i + step] - ts[i]
        if dt <= 0:
            continue
        dx = xs[i + step] - xs[i]
        dy = ys[i + step] - ys[i]
        dist = math.hypot(dx, dy)
        path += dist
        velocities.append(dist / dt)

    # accelerations from velocities
    for i in range(0, len(velocities) - 1):
        # approximate dt using corresponding timestamps
        if i + 1 + step < len(ts):
            dt = ts[i + 1 + step] - ts[i + step]
        else:
            dt = ts[-1] - ts[-1 - step]
        if dt <= 0:
            continue
        a = (velocities[i + 1] - velocities[i]) / dt
        accelerations.append(a)

    # jerks from accelerations
    for i in range(0, len(accelerations) - 1):
        if i + 1 + step < len(ts):
            dt = ts[i + 1 + step] - ts[i + step]
        else:
            dt = ts[-1] - ts[-1 - step]
        if dt <= 0:
            continue
        j = (accelerations[i + 1] - accelerations[i]) / dt
        jerks.append(j)

    return {
        "path_length": path,
        "mean_speed": st.mean(velocities) if velocities else 0.0,
        "speed_std": st.pstdev(velocities) if len(velocities) > 1 else 0.0,
        "jerk_rms": (sum(j * j for j in jerks) / len(jerks)) ** 0.5 if jerks else 0.0,
    }


last_score = None  # for smoothing the stability score


def stability_worker(stability_buffer: Queue, stop_event):
    global last_score
    history = []  # list of (t, x, y)

    while not stop_event.is_set():
        try:
            t, x, y = stability_buffer.get(timeout=0.1)
        except:
            continue
        history.append((t, x, y))

        # keep only last 5 seconds
        t_now = history[-1][0]
        while history and t_now - history[0][0] > 5.0:
            history.pop(0)

        # compute metrics if enough samples
        if len(history) > 5:
            metrics = compute_stability(history)
            if metrics is None:
                continue

            J = metrics['jerk_rms']
            L = metrics['path_length']

            # Path-length gate:
            # - If motion is small (L < 5), use jerk strongly.
            # - If motion is larger, don't penalize as heavily (slow smooth movement).
            if L < 5.0:
                raw_score = 1.0 / (J / 1000.0 + 1e-9) * 100.0
            else:
                # allow more jerk during movement, minimum moderate stability
                raw_score = max(20.0, 1.0 / (J / 3000.0 + 1e-9) * 100.0)

            # clamp to [0, 100]
            raw_score = max(0.0, min(raw_score, 100.0))

            # smooth the score so it doesn't jump frame-to-frame
            if last_score is None:
                smoothed = raw_score
            else:
                alpha = 0.2  # smoothing factor for the score
                smoothed = alpha * raw_score + (1 - alpha) * last_score
            last_score = smoothed

            socketio.emit('stability_update', {'metric': round(smoothed, 1)})

        time.sleep(0.1)


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
        args=(joint_buf, stop_event,),
        daemon=True
    )
    t5 = threading.Thread(
        target=stability_worker,
        args=(stability_buf, stop_event),
        daemon=True
    )

    t1.start()
    t3.start()
    t4.start()
    t5.start()

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
        if video_writer is not None:
            video_writer.release()
        print("Exited cleanly.")


if __name__ == "__main__":
    main()
