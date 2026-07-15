/*
 * BOZGUN - reference Arduino firmware
 * --------------------------------------------------------------------------
 * Drives the pan/tilt servos and the laser module of the BOZGUN turret,
 * driven by the Python control software (bozgun.py).
 *
 * Serial protocol (one command per line):
 *
 *     "<pan>,<tilt>,<laser>\n"      e.g.  "90,90,0\n"
 *
 *       pan   : pan servo angle   (0-180, degrees)
 *       tilt  : tilt servo angle  (0-180, degrees)
 *       laser : 0 = off, 1 = on
 *
 * Baud rate: 9600 (must match BAUDRATE in config.py).
 *
 * NOTE: this is a reference implementation of the firmware side. Adjust the
 * pin numbers below to match your own wiring. Use a low-power laser and follow
 * eye-safety rules. Education / simulation project - not a weapon system.
 */

#include <Servo.h>

const uint8_t PAN_PIN   = 9;   // pan servo signal pin
const uint8_t TILT_PIN  = 10;  // tilt servo signal pin
const uint8_t LASER_PIN = 7;   // laser module control pin

Servo panServo;
Servo tiltServo;

String buffer = "";

void applyCommand(const String &line) {
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) {
    return;  // malformed line, ignore
  }

  int pan   = line.substring(0, c1).toInt();
  int tilt  = line.substring(c1 + 1, c2).toInt();
  int laser = line.substring(c2 + 1).toInt();

  pan  = constrain(pan, 0, 180);
  tilt = constrain(tilt, 0, 180);

  panServo.write(pan);
  tiltServo.write(tilt);
  digitalWrite(LASER_PIN, laser ? HIGH : LOW);
}

void setup() {
  Serial.begin(9600);

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  pinMode(LASER_PIN, OUTPUT);

  // safe start: centred and laser off
  panServo.write(90);
  tiltServo.write(90);
  digitalWrite(LASER_PIN, LOW);
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      applyCommand(buffer);
      buffer = "";
    } else if (c != '\r') {
      buffer += c;
    }
  }
}
