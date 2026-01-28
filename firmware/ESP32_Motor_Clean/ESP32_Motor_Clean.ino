/**
 * ESP32 Motor Control - Clean Implementation
 *
 * Key principles:
 *   - Trust SimpleFOC's shaft_angle tracking (don't override it)
 *   - Use shortest-path for target calculation
 *   - Keep it simple
 *
 * Commands:
 *   c        - Calibrate motor
 *   e        - Enable motor
 *   d        - Disable motor
 *   m <deg>  - Move to position (0-360)
 *   s        - Status
 *   h        - Help
 */

#include "motor_control.h"

MotorController motor;

// Heartbeat timing
unsigned long last_heartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("\n=== ESP32 Motor Control (Clean) ===");
    Serial.println("Commands: c(calibrate) e(enable) d(disable) m<deg> s(status) h(help)\n");

    motor.begin();
}

void loop() {
    // Run motor control loop
    motor.update();

    // Handle serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        handleCommand(cmd);
    }

    // Heartbeat
    if (millis() - last_heartbeat > HEARTBEAT_INTERVAL) {
        last_heartbeat = millis();
        printHeartbeat();
    }
}

void handleCommand(String cmd) {
    if (cmd.length() == 0) return;

    char c = cmd.charAt(0);

    if (c == 'h' || cmd == "help") {
        printHelp();
    }
    else if (c == 'c' || cmd == "calibrate") {
        motor.calibrate();
    }
    else if (c == 'e' || cmd == "enable") {
        motor.enable();
    }
    else if (c == 'd' || cmd == "disable") {
        motor.disable();
    }
    else if (c == 'm' || cmd.startsWith("move")) {
        // Parse angle: "m 90" or "m90" or "move 90"
        float angle = 0;
        int spaceIdx = cmd.indexOf(' ');
        if (spaceIdx > 0) {
            angle = cmd.substring(spaceIdx + 1).toFloat();
        } else if (cmd.length() > 1 && c == 'm') {
            angle = cmd.substring(1).toFloat();
        }
        motor.moveTo(angle);
    }
    else if (c == 's' || cmd == "status") {
        printStatus();
    }
    else if (cmd == "diag") {
        runDiagnostic();
    }
    else {
        Serial.printf("Unknown: '%s' (try 'h' for help)\n", cmd.c_str());
    }
}

void printHelp() {
    Serial.println("\n=== Commands ===");
    Serial.println("  c / calibrate  - Run motor calibration");
    Serial.println("  e / enable     - Enable motor");
    Serial.println("  d / disable    - Disable motor");
    Serial.println("  m <deg>        - Move to position (0-360)");
    Serial.println("  s / status     - Show status");
    Serial.println("  diag           - Run diagnostic");
    Serial.println("  h / help       - This help\n");
}

void printStatus() {
    float enc = motor.getPositionDeg();
    float shaft = motor.getShaftAngleDeg();
    float vel = motor.getVelocityDegS();
    float target = motor.getTargetDeg();

    Serial.println("\n=== Status ===");
    Serial.printf("Encoder:    %.2f° (ground truth)\n", enc);
    Serial.printf("shaft_angle:%.2f° (SimpleFOC internal)\n", shaft);
    Serial.printf("Velocity:   %.1f°/s\n", vel);
    Serial.printf("Target:     %.2f°\n", target);
    Serial.printf("Enabled:    %s\n", motor.isEnabled() ? "YES" : "NO");
    Serial.printf("Calibrated: %s\n", motor.isCalibrated() ? "YES" : "NO");
    Serial.printf("At target:  %s\n\n", motor.isAtTarget() ? "YES" : "NO");
}

void printHeartbeat() {
    float enc = motor.getPositionDeg();
    float shaft = motor.getShaftAngleDeg();
    Serial.printf("[HB] Enc:%.1f° Shaft:%.1f° Vel:%.1f°/s | %s %s\n",
        enc, shaft, motor.getVelocityDegS(),
        motor.isEnabled() ? "EN" : "DIS",
        motor.isAtTarget() ? "AT_TGT" : "");
}

// Simple diagnostic
void runDiagnostic() {
    Serial.println("\n=== Diagnostic ===");

    BLDCMotor& m = motor.getMotor();
    MT6701Sensor& enc = motor.getEncoder();

    // Check encoder
    enc.update();
    Serial.printf("Encoder: %.2f° (raw:%d)\n", enc.getDegrees(), enc.getRawCount());

    // Check SimpleFOC state
    Serial.printf("SimpleFOC: shaft=%.1f° vel=%.1f°/s\n",
        m.shaft_angle * 180/PI, m.shaft_velocity * 180/PI);
    Serial.printf("Config: dir=%s zero=%.1f° vlim=%.1fV\n",
        m.sensor_direction == Direction::CW ? "CW" : "CCW",
        m.zero_electric_angle * 180/PI,
        m.voltage_limit);

    if (!motor.isCalibrated()) {
        Serial.println("\nNot calibrated. Run 'c' first.\n");
        return;
    }

    // Test movement if calibrated
    Serial.println("\nTesting +30° move...");
    motor.enable();
    delay(100);

    float start = enc.getDegrees();
    float target = fmod(start + 30, 360);
    motor.moveTo(target);

    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
        motor.update();
        delay(10);
        if (motor.isAtTarget()) break;
    }

    enc.update();
    float end = enc.getDegrees();
    float moved = end - start;
    if (moved < -180) moved += 360;
    if (moved > 180) moved -= 360;

    Serial.printf("Moved: %.1f° (expected: 30°)\n", moved);
    if (abs(moved - 30) < 5) {
        Serial.println("PASS: Motor working!\n");
    } else if (abs(moved) < 5) {
        Serial.println("FAIL: Motor didn't move\n");
    } else {
        Serial.printf("PARTIAL: Moved %.1f° (check PID)\n\n", moved);
    }
}
