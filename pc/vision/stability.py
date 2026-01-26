import math
import statistics as st
from queue import Queue
import time
from flask_socketio import SocketIO

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


def stability_worker(stability_buffer: Queue, socketio: SocketIO, stop_event):
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