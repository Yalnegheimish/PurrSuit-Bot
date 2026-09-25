/*
 * uno_cat_toy_firmware.ino
 * my Elegoo car v4 -> cat toy. the laptop does the thinking (camera + yolo),
 * this sketch just drives the motors and makes sure we don't ram the couch.
 *
 * pins (from the elegoo shield schematic, TB6612 version):
 *   right motors: PWMA=5, AIN1=7
 *   left motors:  PWMB=6, BIN1=8
 *   driver enable: STBY=3
 *   servo: 10  (wiggles the toy)
 *   ultrasonic on the FRONT: trig=13, echo=12
 *
 * talks over Serial at 9600 -- NOT faster! the esp32 camera board bridges
 * wifi<->serial at 9600 (Serial2.begin(9600) in their code), took me a
 * while to figure that one out...
 *
 * commands coming in (same json style as the stock elegoo firmware):
 *   {"N":102,"D1":d,"D2":s}   drive. d: 1 fwd 2 back 3 left 4 right 9 stop
 *   {"N":5,"D1":1,"D2":a}     servo to angle a (toy wiggle)
 *   {"N":100} or {"N":110}    stop everything
 *   {"H":"x","N":21,"D1":2}   asks for distance, we answer {_x_<cm>}
 *
 * needs the ArduinoJson library (library manager -> "ArduinoJson")
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Servo.h>

// pins
#define PWMA 5
#define PWMB 6
#define AIN1 7
#define BIN1 8
#define STBY 3
#define TRIG 13
#define ECHO 12
#define SERVO_PIN 10

// tweakables
#define SAFE_CM        22    // closer than this while going forward = NOPE
#define CAUTION_CM     40    // slow down zone
#define CMD_TIMEOUT_MS 800   // laptop went quiet -> stop (wifi died?)

Servo toyServo;
unsigned long lastCmd = 0, lastPing = 0;
int curDir = 0, curSpeed = 0;
float frontCm = 400;         // smoothed ultrasonic reading

// ---- motors ----
void motors(int rightSpd, int leftSpd) {   // -255..255 each side
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, rightSpd >= 0 ? HIGH : LOW);
  digitalWrite(BIN1, leftSpd  >= 0 ? HIGH : LOW);
  analogWrite(PWMA, constrain(abs(rightSpd), 0, 255));
  analogWrite(PWMB, constrain(abs(leftSpd),  0, 255));
}

void halt() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  curDir = 0;
}

void go(int dir, int spd) {
  spd = constrain(spd, 0, 255);
  switch (dir) {
    case 1: motors( spd,  spd); break;  // forward
    case 2: motors(-spd, -spd); break;  // back
    case 3: motors( spd, -spd); break;  // spin left
    case 4: motors(-spd,  spd); break;  // spin right
    default: halt(); return;            // 9 or anything weird = stop
  }
  curDir = dir;
  curSpeed = spd;
}

// ---- ultrasonic ----
float pingCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long t = pulseIn(ECHO, HIGH, 25000UL);  // give up past ~4m
  if (t == 0) return 400;   // nothing there (or too far to care)
  return t * 0.0343 / 2.0;
}

// ---- incoming json ----
String rxBuf;

void handleJson(const String &line) {
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, line)) return;  // garbage in -> ignore
  lastCmd = millis();
  int n = doc["N"] | 0;
  const char* id = doc["H"] | "";

  if (n == 102) {
    go(doc["D1"] | 0, doc["D2"] | 0);
  } else if (n == 100 || n == 110) {
    halt();
  } else if (n == 5) {
    toyServo.write(constrain((int)(doc["D2"] | 90), 0, 180));
  } else if (n == 21) {
    // laptop wants the distance
    Serial.print("{_"); Serial.print(id); Serial.print("_");
    Serial.print((int)frontCm); Serial.println("}");
  }
}

void setup() {
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  toyServo.attach(SERVO_PIN);
  toyServo.write(90);
  Serial.begin(9600);   // MUST match the esp32 bridge, see note up top
  halt();
}

void loop() {
  // read whatever the esp32 forwards us, frame by { }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '}') { rxBuf += c; handleJson(rxBuf); rxBuf = ""; }
    else if (c == '{') rxBuf = "{";
    else if (rxBuf.length()) { rxBuf += c; if (rxBuf.length() > 120) rxBuf = ""; }
  }

  // check the front every 60ms
  if (millis() - lastPing > 60) {
    lastPing = millis();
    frontCm = 0.7 * frontCm + 0.3 * pingCm();  // bit of smoothing

    // safety! only matters when driving forward
    if (curDir == 1) {
      if (frontCm < SAFE_CM) {
        // too close -> stop, back up, turn somewhere else, all on our own.
        // the laptop is ~200ms behind reality over wifi, can't trust it here
        halt(); delay(50);
        go(2, 140); delay(250);
        go((millis() & 1) ? 3 : 4, 150); delay(300);  // coin-flip turn
        halt();
        Serial.println("{_evade_1}");  // tell the laptop what happened
      } else if (frontCm < CAUTION_CM && curSpeed > 120) {
        go(1, 120);  // creep, don't charge into stuff
      }
    }
  }

  // laptop gone quiet? (wifi drop, script crashed...) don't keep driving
  if (curDir != 0 && millis() - lastCmd > CMD_TIMEOUT_MS) halt();
}
