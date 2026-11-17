<img width="810" height="308" alt="robot car_v3" src="https://github.com/user-attachments/assets/2fd55cfa-1224-4232-86a1-5cf4d11264c8" />
<h1 align="center">Purrsuit</h1>
An autonomous robot car to play chase with cats. Uses YOLO detection and monocular
distance estimation keep the wand toy just out of the paw's reach.

\
The robot car's ESP32 camera is mounted on the back of the car watching the cat, with a wand toy trailing behind it. When a cat closes in, the car runs away so the cat can chase the toy. The front ultrasonic sensor keeps it from driving into furniture or other obstacles.

## How it works

- *Computer vision detection*: YOLO11s model detects the cat from the rear camera stream.
- *Distance calculation*: Distance is estimated from the height of the
  cat's bounding box and the camera's focal length (pinhole model).
- *Decision loop*:

  - Cat very close (sprint) -> sprint away to avoid capture.
  
  - Near but not close enough (flee) -> flee to keep the cat chasing.
  
  - Mid-range distance (tease) -> zigzag to entice the cat to chase.
  
  - No cat detected (spin) -> step-and-stare search (acts as a sort of enticer as well).
  
- *Obstacle avoidance*: Uses stock ultrasonic sensor in the front to stop and evade obstacles on its own.
 
## Demo video
 
## Repo layout

| Path | Description |
|---|---|
| `purrsuit.py` | Main code, includes detection, decision loop, web dashboard |
| `purrsuit_uno_firmware/` | Arduino sketch for the UNO (motors, ultrasonic logic) |
| `stl/purrsuit_esp32_cam_bracket.obj` | 3D-printable rear camera bracket |
| `stl/purrsuit_wand_holder.obj` | 3D-printable wand toy holder |
| `requirements.txt` | Python dependencies |

## BOM

| Part | Qty | Notes |
|---|---|---|
| ELEGOO Smart Robot Car Kit V4.0 with camera | 1 | Base car, had all the basic functions I was looking for |
| Printed camera bracket | 1 | `stl/purrsuit_esp32_cam_bracket.obj` |
| Printed wand holder | 1 | `stl/purrsuit_wand_holder.obj` |
| Cat wand toy | 1 | Personally used the wand portion of [this toy](https://smartykat.com/products/smartykat%C2%AE-hot-pursuit%E2%84%A2-electronic-concealed-motion-electronic-motion-cat-toy?_pos=1&_psid=7be26325e&_psq=hot%20pursuit&_ss=e) that I already had due to it's perfect length |
| M2*8 Screws | 2 | To attach the esp32 cam to the custom mount |
| M4*16 Screws | 3 | One to attach the cam custom mount to the top acrylic base and two to attach the custom wand mount to the bottom acrylic base |
| M4 Nuts | 3 | To secure the two mounts |
| Cat(s) | 1+ | Tested with 2 |

## Setup

### 1. Hardware
1. Print the two mounts and install the camera bracket at the rear of the top plate (camera facing backward) and the wand holder at the rear of the bottom plate. Holes will need to be drilled as per the below photo.

<img width="1731" height="856" alt="bottom plate holes" src="https://github.com/user-attachments/assets/45694865-0f8d-41fb-92df-293ca194b504" />
<img width="1852" height="993" alt="top plate holes" src="https://github.com/user-attachments/assets/64896f16-c5bd-40e2-8ef8-152dd34a532b" />



2. Assemble the rest of the car per the ELEGOO manual.

3. Attach a wand to the holder and use a zip tie to secure the toy to it.

<img width="2560" height="1920" alt="image" src="https://github.com/user-attachments/assets/eff36a2e-4946-4ec5-8c40-e637d102105e" />

 
### 2. Firmware

- *Arduino UNO*: open `purrsuit_uno_firmware/` in the Arduino IDE, install the "ArduinoJson" library, slide the car's mode switch to upload, flash the firmware, then slide it back.
 
### 3. Code

1. Install requirements:
```bash
pip install -r requirements.txt
```

2. Adjust parameters:

Adjust the parameters in `purrsuit.py`. See ['Adjustable Parameters'](#options) section.
 


3. Run the code:

Connect to the car's stock ELEGOO-xxxx WiFi network, then:

```bash
python purrsuit.py --calibrate   # first time: tune the distance estimate
python purrsuit.py               # run the code
```

Open *http://localhost:3333* to watch the live feed: detections, estimated
distance, the decision the robot is making, and a log of its
behavior.

## Options
1. Flags:


| Flag | Use |
|---|---|
| `--calibrate` | Motors off, video + distance readout only |
| `--record` | Saves a frame every 2 s to cat_dataset/ for fine-tuning |
| `--ip <addr>` | Robot IP override if not the default `192.168.4.1` |
| `--stream <url>` | Force a specific video stream URL instead of the default `http://192.168.4.1:81/stream` |

2. Adjustable parameters in `purrsuit.py`: 


| **Parameter** | **Description** |
|---|---|
| `FOCAL_RATIO` | For finding the focal length. See the [calibration](#calibrating-distance) section to calibrate this value |
| `CAT_HEIGHT` | Cat's standing height; used to estimate distance |
| `POUNCE_DIST` | Distance where the cat is on the toy (Wand toy length) |
| `CHASE_DIST` | Distance where the cat is considered chasing the car |
| `TEASE_DIST` | Distance where the cat is a bit further and watching the car |
| `SPRINT_SPD` | Sprinting speed (fast) |
| `FLEE_SPD` | Fleeing speed (slower) |
| `TEASE_SPD` | Teasing speed (slowest) |
| `SEARCH_SPIN_SPD` | Search spinning speed (keep above 110) |
| `SEARCH_SPIN_SEC` | Spin step length in seconds |
| `SEARCH_LOOK_SEC` | Step hold length in seconds (needed due to latency) |
| `MODEL_FILE` | CV model |
| `CONF` | Confidence threshold |
| `DETECT_SIZE` | Inference size |
| `CAT_CLASSES` | COCO Classes |
| `HOLD_SEC` | Action decision hold length in seconds |
| `ZIG_FWD_SEC` | Forward motion length in seconds |
| `ZIG_TURN_SEC` | Length of each zigzag in seconds |
| `ZIG_TURN_SPD` | Zigzag turn speed |
| `PLAY_TIME` | Seconds of play before a break in seconds |
| `BREAK_TIME` | Length of the break time in seconds |

### Calibrating distance

1. Run with `--calibrate` (motors stay off in this mode).
2. Place a cat-sized object (~25 cm tall) exactly 1 m behind the car.
3. Adjust `FOCAL_RATIO` in `purrsuit.py` until the dashboard reads 1.00 m.
4. If your cats are bigger or smaller than 25 cm at the shoulder,
   adjust `CAT_HEIGHT` too.
 
## Future improvements

- Experiment with different distance estimation algorithms. 
 
- Experiment with sensors to improve object detection. Currently the car avoids obstacles only if it's very close to them and can't detect anything lower than the top plate.
 
- Find an inexpensive board to move detection onto and reduce detection and decision latency.
 
- Experiment with different path planning algorithms around obstacles instead of just a straight line.

## License

MIT — see [LICENSE](LICENSE).

Uses [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
(AGPL-3.0). ELEGOO's stock firmware can be found on
[ELEGOO's site](https://www.elegoo.com/blogs/arduino-projects/elegoo-smart-robot-car-kit-v4-0-tutorial).
