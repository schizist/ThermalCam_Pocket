// Servo Type Diagnostic
// Determines: 360-degree continuous rotation vs standard positional servo
//
// How it works:
//   - Standard positional servo: moves to the commanded angle and holds there.
//   - 360 CR servo:              90 deg (1500 us) = STOP, <90 = spin one way, >90 = spin other.
//   - Encoder servo:             behaves like positional but may have a feedback pin.
//
// Wiring: connect servo signal wire to TEST_PIN (default GPIO13).
//         Change TEST_PIN below if using a different pin.
//
// Usage: open Serial Monitor at 115200.
//        The auto sequence runs on boot, then interactive commands are available.

#include <Arduino.h>

#define TEST_PIN 13   // GPIO for servo signal — change to 32 for the eyelid servo

static const uint32_t SERVO_FREQ       = 50;
static const uint8_t  SERVO_RESOLUTION = 16;
static const float    SERVO_MIN_US     = 500.0f;
static const float    SERVO_MAX_US     = 2500.0f;
static const float    SERVO_PERIOD_US  = 20000.0f;
static const uint32_t MAX_DUTY        = (1u << SERVO_RESOLUTION) - 1;

static void servoWriteUs(float us) {
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t duty = (uint32_t)(us / SERVO_PERIOD_US * (float)MAX_DUTY);
  ledcWrite(TEST_PIN, duty);
}

static void servoWriteDeg(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  float us = SERVO_MIN_US + (deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  servoWriteUs(us);
}

static void servoOff() {
  ledcWrite(TEST_PIN, 0);  // coast — no PWM pulse
}

static void step(const char *msg, float deg, uint32_t holdMs) {
  float us = SERVO_MIN_US + (deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  Serial.printf("\n[STEP] %s\n", msg);
  Serial.printf("       Sending %.0f deg  (%.0f us)\n", deg, us);
  Serial.printf("       >> OBSERVE the servo for %lu ms <<\n", holdMs);
  servoWriteDeg(deg);
  delay(holdMs);
}

static void printHelp() {
  Serial.println("\n--- Commands ---");
  Serial.println("  0..180  : send that angle (e.g. \"90\" sets 90 deg)");
  Serial.println("  us NNNN : send raw pulse in microseconds (e.g. \"us 1500\")");
  Serial.println("  off     : cut PWM signal (servo coasts/de-energises)");
  Serial.println("  sweep   : sweep 0→180→0 slowly");
  Serial.println("  help    : show this list");
  Serial.println("----------------");
}

static void runAutoSequence() {
  Serial.println("\n=============================================");
  Serial.println("  SERVO TYPE DIAGNOSTIC — auto sequence");
  Serial.println("=============================================");
  Serial.println("Watch the servo carefully during each step.");
  Serial.println();
  Serial.println("LEGEND (what to look for):");
  Serial.println("  POSITIONAL servo  -> snaps to the stated angle and holds still.");
  Serial.println("  360-CR servo      -> at 90 deg it STOPS;");
  Serial.println("                       below 90 spins one direction,");
  Serial.println("                       above 90 spins the other.");
  Serial.println("  Encoder servo     -> same behaviour as positional from the outside;");
  Serial.println("                       check datasheet/markings for feedback wires.");
  Serial.println();
  delay(2000);

  // Step 1: neutral — CR servo should STOP here
  step("NEUTRAL (90 deg = 1500 us)", 90.0f, 3000);
  Serial.println("       Did it STOP spinning? (Yes = CR servo)");
  Serial.println("       Did it move to mid-position? (Yes = positional servo)");
  delay(1500);

  // Step 2: minimum — CR servo spins one way, positional goes to min angle
  step("MINIMUM (0 deg = 500 us)", 0.0f, 3000);
  Serial.println("       Spinning continuously? (Yes = CR servo)");
  Serial.println("       Moved to hard stop / min position? (Yes = positional)");
  delay(1500);

  // Step 3: maximum — CR servo spins opposite way
  step("MAXIMUM (180 deg = 2500 us)", 180.0f, 3000);
  Serial.println("       Spinning the OTHER direction? (Yes = CR servo)");
  Serial.println("       Moved to opposite hard stop? (Yes = positional)");
  delay(1500);

  // Step 4: back to neutral — confirms CR by stopping again
  step("NEUTRAL again (90 deg = 1500 us)", 90.0f, 3000);
  Serial.println("       Stopped again? (Yes = CR servo confirmed)");
  delay(1500);

  // Step 5: mid-range positional check — positional servo should hold angle
  step("QUARTER (45 deg = 875 us)", 45.0f, 3000);
  Serial.println("       Moved to a distinct position between min and mid? (positional)");
  Serial.println("       Still spinning (slower)? (CR servo)");
  delay(1500);

  // Step 6: coast — both types lose hold
  Serial.println("\n[STEP] COAST — cutting PWM signal");
  Serial.println("       Servo should go limp / de-energise (both types).");
  servoOff();
  delay(2000);

  Serial.println("\n=============================================");
  Serial.println("  AUTO SEQUENCE COMPLETE");
  Serial.println("  RESULT GUIDE:");
  Serial.println("    90 deg = stops, 0/180 deg = spins  => 360-degree CR servo");
  Serial.println("    Moves to fixed angle at each step  => Standard positional servo");
  Serial.println("    Positional + extra signal wires    => Encoder/feedback servo");
  Serial.println("=============================================");

  printHelp();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\nServo test on GPIO%d\n", TEST_PIN);

  ledcAttach(TEST_PIN, SERVO_FREQ, SERVO_RESOLUTION);
  servoWriteDeg(90.0f);  // safe neutral before sequence
  delay(500);

  runAutoSequence();
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "help") {
    printHelp();
    return;
  }

  if (cmd == "off") {
    servoOff();
    Serial.println("[CMD] PWM off (servo coasting)");
    return;
  }

  if (cmd == "sweep") {
    Serial.println("[CMD] Sweep 0->180->0 ...");
    for (float a = 0; a <= 180; a += 2) {
      servoWriteDeg(a);
      Serial.printf("  %.0f deg\r", a);
      delay(30);
    }
    for (float a = 180; a >= 0; a -= 2) {
      servoWriteDeg(a);
      Serial.printf("  %.0f deg\r", a);
      delay(30);
    }
    servoWriteDeg(90);
    Serial.println("\n[CMD] Sweep done — returned to 90 deg");
    return;
  }

  if (cmd.startsWith("us ")) {
    float us = cmd.substring(3).toFloat();
    if (us < 400 || us > 2600) {
      Serial.println("[ERR] Pulse out of range (400..2600 us)");
      return;
    }
    servoWriteUs(us);
    Serial.printf("[CMD] %.0f us sent\n", us);
    return;
  }

  // Plain number = angle
  float angle = cmd.toFloat();
  if (angle == 0 && cmd != "0") {
    Serial.println("[ERR] Unknown command. Type 'help'.");
    return;
  }
  angle = constrain(angle, 0, 180);
  servoWriteDeg(angle);
  float us = SERVO_MIN_US + (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  Serial.printf("[CMD] %.1f deg  (%.0f us)\n", angle, us);
}
