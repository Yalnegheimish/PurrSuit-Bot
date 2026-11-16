# How to run:
#   pip install -r requirements.txt
#   connect the machine to the ELEGOO-XXXX wifi (no password)
#   python purrsuit.py                 <- play mode
#   python purrsuit.py --calibrate     <- motors OFF, for tuning distance
#   python purrsuit.py --ip 1.2.3.4    <- if robot isn't at 192.168.4.1
#   python purrsuit.py --record        <- for recording fine-tuning dataset
#
# then open  http://localhost:3333  in a browser to see what the robot sees
# and what it's deciding to do. 

import argparse
import json
import socket
import threading
import time
import urllib.request
from collections import deque

import cv2
import numpy as np
from flask import Flask, Response
from ultralytics import YOLO

# ----------------- settings -----------------
ROBOT_IP   = "192.168.4.1"   # robot's AP address
CTRL_PORT  = 100             # elegoo tcp command port
STREAM_URLS = [              
    "http://{ip}:81/stream",  
    "http://{ip}/Test",       
]

FOCAL_RATIO = 0.79  # focal length = this * frame width (65 deg lens)
                      # to calibrate: put something cat-sized exactly 1m
                      # behind the car, run --calibrate, scale this number
                      # until the feed reads 1.00
CAT_HEIGHT  = 0.25    # cat standing height

# distances in meters (cat -> rear camera)
POUNCE_DIST = 0.45    # cat's basically on the toy -> run
CHASE_DIST  = 1.10    # cat's chasing -> keep running
TEASE_DIST  = 2.50    # cat's watching -> tempt it

SPRINT_SPD = 220
FLEE_SPD   = 160
TEASE_SPD  = 110

SEARCH_SPIN_SPD  = 130    # search spin speed (keep it above 110)
SEARCH_SPIN_SEC  = 0.2   # how long each little spin step lasts
SEARCH_LOOK_SEC  = 0.9    # how long we hold still and actually look

MODEL_FILE  = "yolo11s.pt"   # CV model
CONF        = 0.4           # confidence threshold
DETECT_SIZE = 640            # inference size
CAT_CLASSES = [15, 16]       # 15=cat 16=dog
HOLD_SEC    = 1.0            # keep chasing on the last known position this
                             # long when detection flickers out for a frame


ZIG_FWD_SEC  = 0.45   # forward leg of the zigzag
ZIG_TURN_SEC = 0.18   # how long each zigzag lasts
ZIG_TURN_SPD = 150    # zigzag turn speed


PLAY_TIME  = 300      # seconds of play before a forced break
BREAK_TIME = 60       # let the cat "win" 

# ----------------- shared state for the feed -----------------
STATE = {
    "decision": "starting up...",
    "dist": None,
    "mode": "play",
    "evades": 0,          # ultrasonic sensor evasion
    "linked": False,
}
latest_jpg = None         # last annotated frame as jpeg bytes
frame_lock = threading.Lock()

# rolling event log for the web pag
LOG = deque(maxlen=150)

def log_event(action, dist=None, conf=None):
    LOG.append({
        "t": time.strftime("%H:%M:%S"),
        "action": action,
        "dist": None if dist is None else round(float(dist), 2),
        "conf": None if conf is None else round(float(conf), 2),
    })

# ----------------- robot tcp link -----------------
# the esp32 sends {Heartbeat} every second and DROPS US if we don't send
# {Heartbeat} back 
sock = None
sock_lock = threading.Lock()

def sock_connect(ip):
    global sock
    try:
        s = socket.create_connection((ip, CTRL_PORT), timeout=3)
        s.settimeout(0.25)
        sock = s
        STATE["linked"] = True
        log_event("link up")
        print("[link] connected!")
    except OSError as e:
        sock = None
        STATE["linked"] = False
        print("[link] can't connect:", e, "-- are you on the robot's wifi?")

def send_raw(text):
    global sock
    with sock_lock:
        if sock is None:
            return
        try:
            sock.sendall(text.encode())
        except OSError:
            try: sock.close()
            except OSError: pass
            sock = None
            STATE["linked"] = False

def send_cmd(obj):
    send_raw(json.dumps(obj, separators=(",", ":")))

def drive(direction, speed):   # 1 fwd, 2 back, 3 spin left, 4 spin right from firmware
    send_cmd({"N": 102, "D1": direction, "D2": int(speed)})

def stop_car():
    send_cmd({"N": 102, "D1": 9, "D2": 0})   # 9 = stop+standby

def set_servo(angle):
    # center servo
    send_cmd({"N": 5, "D1": 1, "D2": int(angle)})


def link_babysitter(ip):
    """runs forever in a thread: answers heartbeats, reconnects if dropped"""
    global sock
    buf = b""
    while True:
        if sock is None:
            time.sleep(1)
            sock_connect(ip)
            continue
        try:
            data = sock.recv(256)
            if not data:            # esp32 hung up 
                sock = None
                STATE["linked"] = False
                continue
            buf += data
            while b"}" in buf:
                part, _, buf = buf.partition(b"}")
                msg = part.decode(errors="ignore").lstrip("{").strip()
                if msg == "Heartbeat":
                    send_raw("{Heartbeat}")     
                elif msg == "_evade_1":
                    STATE["evades"] += 1
                    log_event("Ultrasonic sensor evasion")
                    print("[car] dodged an obstacle on its own")
                # everything else ({ok} etc) we just ignore
        except socket.timeout:
            pass
        except OSError:
            sock = None
            STATE["linked"] = False

# ----------------- camera -----------------
def open_camera(ip, forced_url=None):
    urls = [forced_url] if forced_url else [u.format(ip=ip) for u in STREAM_URLS]
    for url in urls:
        cap = cv2.VideoCapture(url)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                print("[cam] streaming from", url)
                return cap, None
        cap.release()
    # plan B: grab single jpegs from /capture in a loop.
    cap_url = f"http://{ip}/capture"
    try:
        data = urllib.request.urlopen(cap_url, timeout=3).read()
        if cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR) is not None:
            print("[cam] falling back to polling", cap_url)
            return None, cap_url
    except Exception:
        pass
    raise SystemExit("[cam] no video :( open http://" + ip + " in a browser, "
                     "find the stream url, pass it with --stream")

def grab_frame(cap, cap_url):
    if cap is not None:
        return cap.read()
    try:
        data = urllib.request.urlopen(cap_url, timeout=2).read()
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        return img is not None, img
    except Exception:
        return False, None

# ----------------- the web page -----------------
app = Flask(__name__)

PAGE = """
<html><head><title>Purrsuit</title>
<style>
 body { background:#1e1e1e; color:#eee; font-family:monospace; text-align:center }
 img  { max-width:95%; border:2px solid #444; margin-top:10px }
 #d   { font-size:1.4em; margin:10px }
 .cal { color:#ffb347 }
 #log { max-width:820px; height:185px; margin:12px auto; padding:8px;
        background:#111; border:1px solid #444; border-radius:4px;
        overflow-y:auto; text-align:left; white-space:pre; font-size:0.85em;
        color:#9ecb9e }
</style></head><body>
<h2>Live Purrsuit feed</h2>
<div id="d">...</div>
<img src="/video">
<div id="log">waiting for events...</div>
<script>
 setInterval(async () => {
   let s = await (await fetch('/status')).json();
   let t = s.decision + (s.dist ? ' | cat at ' + s.dist.toFixed(2) + ' m' : '');
   t += ' | link: ' + (s.linked ? 'OK' : 'DOWN') + ' | evades: ' + s.evades;
   let el = document.getElementById('d');
   el.textContent = t;
   el.className = (s.mode == 'calibrate') ? 'cal' : '';

   // behaviour log, newest line on top
   let lines = s.log.slice().reverse().map(e => {
     let x = '[' + e.t + '] ' + e.action;
     if (e.dist != null) x += '  |  dist ' + e.dist.toFixed(2) + ' m';
     if (e.conf != null) x += '  |  conf ' + e.conf.toFixed(2);
     return x;
   });
   document.getElementById('log').textContent = lines.join('\\n');
 }, 400);
</script></body></html>
"""

@app.route("/")
def index():
    return PAGE

@app.route("/status")
def status():
    # deque to list
    return {**STATE, "log": list(LOG)}

@app.route("/video")
def video():
    def gen():
        while True:
            with frame_lock:
                jpg = latest_jpg
            if jpg is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            time.sleep(0.05)   # ~20fps
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ----------------- main -----------------
def main():
    global latest_jpg

    ap = argparse.ArgumentParser()
    ap.add_argument("--calibrate", action="store_true",
                    help="motors off, just show video + distance for tuning")
    ap.add_argument("--ip", default=ROBOT_IP)
    ap.add_argument("--stream", default=None, help="force a stream url")
    ap.add_argument("--record", action="store_true",
                    help="save a frame every 2s to cat_dataset/ so you can "
                         "fine-tune YOLO on your cats") 

    args = ap.parse_args()

    STATE["mode"] = "calibrate" if args.calibrate else "play"

    print("[yolo] loading", MODEL_FILE)
    model = YOLO(MODEL_FILE)

    cap, cap_url = open_camera(args.ip, args.stream)

    threading.Thread(target=link_babysitter, args=(args.ip,), daemon=True).start()
    threading.Thread(target=lambda: app.run(host="0.0.0.0", port=3333,
                     debug=False, use_reloader=False), daemon=True).start()
    print("[web] feed page -> http://localhost:3333")

    time.sleep(1.5)    # give the tcp link a sec to come up
    set_servo(90)      # point the ultrasonic straight ahead and leave it there



    if args.calibrate:
        print("[cal] CALIBRATION MODE - motors are off.")
        print("[cal] put a cat-sized thing at a known distance behind the car,")
        print("[cal] check the number on the feed, adjust FOCAL_RATIO.")
        stop_car()         # just in case it was moving

    last_cmd = 0
    last_seen = 0
    held_dist, held_x = None, None
    last_save = 0 
    session_start = time.time()

    while True:
        ok, frame = grab_frame(cap, cap_url)
        if not ok:
            time.sleep(0.1)
            continue

        if args.record and time.time() - last_save > 2:
            last_save = time.time()
            import os
            os.makedirs("cat_dataset", exist_ok=True)
            cv2.imwrite(f"cat_dataset/{int(last_save)}.jpg", frame)
 

        # --- find the cat ---
        res = model.predict(frame, classes=CAT_CLASSES, conf=CONF,
                            imgsz=DETECT_SIZE, verbose=False)[0]
        dist = None
        x_off = None
        conf = None
        if len(res.boxes) > 0:
            boxes = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            # biggest box = closest cat (matters when both cats show up)
            i = int(np.argmax((boxes[:,2]-boxes[:,0]) * (boxes[:,3]-boxes[:,1])))
            x1, y1, x2, y2 = boxes[i].astype(int)
            conf = float(confs[i])
            f_px = FOCAL_RATIO * frame.shape[1]
            dist = f_px * CAT_HEIGHT / max(y2 - y1, 1)
            x_off = ((x1+x2)/2 - frame.shape[1]/2) / (frame.shape[1]/2)  # -1..1
            last_seen = time.time()
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(frame, f"cat {dist:.2f}m ({conf:.2f})", (x1, max(y1-8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            held_dist, held_x = dist, x_off       # remember for flicker frames
        elif time.time() - last_seen < HOLD_SEC:
            # keep acting on where the cat just was instead of stopping dead
            dist, x_off = held_dist, held_x
            cv2.putText(frame, "(holding last action)", (10, frame.shape[0]-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,200), 2)
        STATE["dist"] = dist

        now = time.time()

        # --- decision tree ---
        if args.calibrate:
            decision = "CALIBRATE (motors off)"
        elif now - session_start > PLAY_TIME:
            decision = "break time"
            if now - session_start > PLAY_TIME + BREAK_TIME:
                session_start = now      # break's over, back to playing
        elif dist is not None and dist < POUNCE_DIST:
            decision = "run! cat's close"
        elif dist is not None and dist < CHASE_DIST:
            decision = "fleeing, keep the chase going"
        elif dist is not None and dist < TEASE_DIST:
            decision = "teasing..."
        elif now - last_seen < 3:
            decision = "cat disappeared, waiting"
        else:
            decision = "no cat, spinning to find one"

        # action log
        if decision != STATE["decision"]:
            log_event(decision, dist, conf)
        STATE["decision"] = decision

        # --- action (rate limited) ---
        if not args.calibrate and now - last_cmd > 0.15:
            last_cmd = now
            if "run" in decision:
                drive(1, SPRINT_SPD)
            elif "fleeing" in decision:
                # camera faces backward so left/right are mirrored
                if x_off is not None and x_off > 0.25:
                    drive(4, FLEE_SPD)
                elif x_off is not None and x_off < -0.25:
                    drive(3, FLEE_SPD)
                else:
                    drive(1, FLEE_SPD)
            elif "teasing" in decision:
                # zigzag: fwd -> jink left -> fwd -> jink right -> repeat.
                cycle = 2 * (ZIG_FWD_SEC + ZIG_TURN_SEC)
                phase = now % cycle
                if phase < ZIG_FWD_SEC:
                    drive(1, TEASE_SPD)
                elif phase < ZIG_FWD_SEC + ZIG_TURN_SEC:
                    drive(3, ZIG_TURN_SPD)
                elif phase < 2*ZIG_FWD_SEC + ZIG_TURN_SEC:
                    drive(1, TEASE_SPD)
                else:
                    drive(4, ZIG_TURN_SPD)
            elif "disappeared" in decision or "break" in decision:
                stop_car()
            else:
                # searching: spin a small step, then hold still and LOOK.
                phase = now % (SEARCH_SPIN_SEC + SEARCH_LOOK_SEC)
                if phase < SEARCH_SPIN_SEC:
                    drive(3, SEARCH_SPIN_SPD)
                else:
                    stop_car()

        # --- stamp the decision and distance on the frame for the feed ---
        cv2.putText(frame, decision, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (50, 200, 255), 2)
        if args.calibrate and dist is not None:
            cv2.putText(frame, f"DIST: {dist:.2f} m", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (80, 180, 255), 3)
        ok2, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok2:
            with frame_lock:
                latest_jpg = jpg.tobytes()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_car()
        time.sleep(0.2)
        print("\ncar stopped.")
