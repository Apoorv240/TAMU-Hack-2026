from flask import Flask, Response, send_from_directory, render_template, jsonify
from flask_socketio import SocketIO, emit
from queue import Queue
import os
from glob import glob
import cv2
import time

from ..vision.camera import FPS

recordings_dir = os.path.join(os.path.dirname(__file__), "recordings")
os.makedirs(recordings_dir, exist_ok=True)

app = Flask(__name__, static_folder='recordings')
socketio = SocketIO(app)

video_writer = None
current_case_id = None
recording_active = False

fourcc = cv2.VideoWriter_fourcc(*'avc1') 

frame_buf = None
stop_event = None

def list_recordings():
    files = sorted(
        glob(os.path.join(recordings_dir, "*.mp4")),
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
                path = os.path.join(recordings_dir, filename)
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
    return send_from_directory(recordings_dir, name, as_attachment=True, mimetype="video/mp4")


@app.route("/play/<name>")
def play_recording(name):
    # simple static file; the front-end can use this as a video src
    return send_from_directory(recordings_dir, name)


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