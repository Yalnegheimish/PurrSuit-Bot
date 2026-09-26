#!/usr/bin/env python3
#
# camera is on the BACK of the car looking at the cat, toy drags behind on
# the wand. when the cat gets close the car runs away so the cat chases the
# toy. front ultrasonic stops it from faceplanting into furniture ideally (that bit
# lives in the Arduino sketch, the car handles it on its own).
#
# how to run:
#   pip install ultralytics opencv-python flask numpy
#   connect the machine to the ELEGOO-XXXX wifi (no password)
#   python cat_chaser.py                 <- play mode
#   python cat_chaser.py --calibrate     <- motors OFF, for tuning distance
#   python cat_chaser.py --ip 1.2.3.4    <- if robot isn't at 192.168.4.1
#
# then open  http://localhost:3333  in a browser to see what the robot sees
# and what it's deciding to do. 

import argparse
import json
import socket
import threading
import time
import urllib.request

import cv2
import numpy as np
from flask import Flask, Response
from ultralytics import YOLO

# ----------------- settings -----------------
ROBOT_IP   = "192.168.4.1"   # robot's AP address
CTRL_PORT  = 100             # elegoo tcp command port
STREAM_URLS = [              
    "http://{ip}:81/stream",  # <- the real one (checked the esp32 source)
    "http://{ip}/Test",       # also streams
]

FOCAL_RATIO = 0.79  # focal length = this * frame width (65 deg lens)
                      # to calibrate: put something cat-sized exactly 1m
                      # behind the car, run --calibrate, scale this number
                      # until the feed reads 1.00
CAT_HEIGHT  = 0.25    # meters, roughly a cat. bigger cat = bigger number

# distances in meters (cat -> rear camera)
POUNCE_DIST = 0.45    # cat's basically on the toy -> RUN
CHASE_DIST  = 1.10    # cat's chasing -> keep running
TEASE_DIST  = 2.50    # cat's watching -> tempt it

SPRINT_SPD = 220
FLEE_SPD   = 160
TEASE_SPD  = 110

# spin a little -> stop -> let the camera get 2-3 sharp frames -> repeat.
SEARCH_SPIN_SPD  = 130    # tank turns need a bit of oomph or they stall (keep it above 110)
SEARCH_SPIN_SEC  = 0.1   # how long each little spin step lasts
SEARCH_LOOK_SEC  = 0.9    # how long we hold still and actually look

MODEL_FILE  = "yolo11s.pt"   # select your model
CONF        = 0.25           # lower = catches more cats, more false alarms
DETECT_SIZE = 640            # inference size. 416 was missing far-away cats
CAT_CLASSES = [15, 16]       # 15=cat 16=dog. yes dog. coco models mix up
                             # cats and dogs ALL the time.
HOLD_SEC    = 1.0            # keep chasing on the last known position this
                             # long when detection flickers out for a frame


PLAY_TIME  = 300      # seconds of play before a forced break
BREAK_TIME = 60       # let the cat "win" and chill for a bit

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

def wiggle_toy(angle):
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
            if not data:            # esp32 hung up on us
                sock = None
                STATE["linked"] = False
                continue
            buf += data
            while b"}" in buf:
                part, _, buf = buf.partition(b"}")
                msg = part.decode(errors="ignore").lstrip("{").strip()
                if msg == "Heartbeat":
                    send_raw("{Heartbeat}")     # the magic keep-alive
                elif msg == "_evade_1":
                    STATE["evades"] += 1
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
    # plan B: grab single jpegs from /capture in a loop. slower but works
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

# ----------------- the web debug page -----------------
app = Flask(__name__)

PAGE = """
<html><head><title>Purrsuit</title>
<style>
 body { background:#1e1e1e; color:#eee; font-family:monospace; text-align:center }
 img  { max-width:95%; border:2px solid #444; margin-top:10px }
 #d   { font-size:1.4em; margin:10px }
 .cal { color:#ffb347 }
</style></head><body>
<h2>Live Purrsuit feed</h2>
<div id="d">...</div>
<img src="/video">
<script>
 setInterval(async () => {
   let s = await (await fetch('/status')).json();
   let t = s.decision + (s.dist ? ' | cat at ' + s.dist.toFixed(2) + ' m' : '');
   t += ' | link: ' + (s.linked ? 'OK' : 'DOWN') + ' | evades: ' + s.evades;
   let el = document.getElementById('d');
   el.textContent = t;
   el.className = (s.mode == 'calibrate') ? 'cal' : '';
 }, 400);
</script></body></html>
"""

@app.route("/")
def index():
    return PAGE

@app.route("/status")
def status():
    return STATE

@app.route("/video")
def video():
    def gen():
        while True:
            with frame_lock:
                jpg = latest_jpg
            if jpg is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            time.sleep(0.05)   # ~20fps, plenty for debugging
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
                         "fine-tune yolo on your cats") 

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
    wiggle_toy(90)     # center ultrasonic sensor once so the crash guard looks straight ahead



    if args.calibrate:
        print("[cal] CALIBRATION MODE - motors are off.")
        print("[cal] put a cat-sized thing at a known distance behind the car,")
        print("[cal] check the number on the feed, adjust FOCAL_RATIO.")
        stop_car()         # just in case it was moving

    last_cmd = 0
    last_seen = 0
    held_dist, held_x = None, None
    last_save = 0
    wig = 0 # wiggle 
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
        if len(res.boxes) > 0:
            boxes = res.boxes.xyxy.cpu().numpy()
            # biggest box = closest cat (matters when both cats show up)
            b = boxes[np.argmax((boxes[:,2]-boxes[:,0]) * (boxes[:,3]-boxes[:,1]))]
            x1, y1, x2, y2 = b.astype(int)
            f_px = FOCAL_RATIO * frame.shape[1]
            dist = f_px * CAT_HEIGHT / max(y2 - y1, 1)
            x_off = ((x1+x2)/2 - frame.shape[1]/2) / (frame.shape[1]/2)  # -1..1
            last_seen = time.time()
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(frame, f"cat {dist:.2f}m", (x1, max(y1-8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            held_dist, held_x = dist, x_off       # remember for flicker frames
        elif time.time() - last_seen < HOLD_SEC:
            # detector blinked (happens a lot with a moving cat) -- keep
            # acting on where the cat just was instead of stopping dead
            dist, x_off = held_dist, held_x
            cv2.putText(frame, "(holding last fix)", (10, frame.shape[0]-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,200), 2)
        STATE["dist"] = dist

        now = time.time()

        # --- decision tree ---
        if args.calibrate:
            decision = "CALIBRATE (motors off)"
        elif now - session_start > PLAY_TIME:
            decision = "break time, cats gotta rest"
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
        STATE["decision"] = decision

        # --- action (rate limited so we don't spam the esp32) ---
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
                drive(1, TEASE_SPD)
                wig = (wig + 1) % 4
                wiggle_toy(60 if wig < 2 else 120)
            elif "vanished" in decision or "break" in decision:
                stop_car()
            else:
                # searching: spin a small step, then hold still and LOOK.
                # spinning nonstop = motion blur = yolo never sees the cat
                phase = now % (SEARCH_SPIN_SEC + SEARCH_LOOK_SEC)
                if phase < SEARCH_SPIN_SEC:
                    drive(3, SEARCH_SPIN_SPD)
                else:
                    stop_car()

        # --- stamp the decision on the frame for the feed ---
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
