/*
  Milestone 8: Obstacle Sensor Calibration
  Board: Arduino UNO R4 WiFi. Serial Monitor: 9600 baud.
  Open only this sketch in its same-named folder. See README.md in the ZIP.
  Kill: D2 -> GND, INPUT_PULLUP; active-LOW power relay on D0.
  Boot waits 10 s. Press once to kill; double press within 1 s to restart
  after 60 s; third press changes that wait to 2 s. Full demo restarts.
*/
#include <Arduino.h>
#if !defined(ARDUINO_UNOR4_WIFI)
#error "Select Arduino UNO R4 WiFi. This pin map and interrupt require that board."
#endif

constexpr unsigned long STARTUP_DELAY_MS = 10000UL;
constexpr uint8_t KILL_SWITCH_PIN = 2;  // UNO R4 external interrupt pin
constexpr uint8_t POWER_RELAY_PIN = 0;
constexpr uint8_t RELAY_ON = LOW;
constexpr uint8_t RELAY_OFF = HIGH;
constexpr unsigned long DOUBLE_PRESS_MS = 1000UL;
constexpr unsigned long REPOSITION_MS = 60000UL;
constexpr unsigned long EARLY_START_MS = 2000UL;
constexpr uint8_t LEFT_MOTORS_PWM_PIN = 5;
constexpr uint8_t LEFT_MOTORS_IN1_PIN = 7;
constexpr uint8_t LEFT_MOTORS_IN2_PIN = 4;
constexpr uint8_t RIGHT_MOTORS_PWM_PIN = 6;
constexpr uint8_t RIGHT_MOTORS_IN1_PIN = 8;
constexpr uint8_t RIGHT_MOTORS_IN2_PIN = 9;
constexpr int DRIVE_SPEED = 25;
constexpr int TURN_SPEED = 25;
constexpr uint8_t FAN_PIN = 1;
constexpr uint8_t COMMON_ULTRASONIC_TRIG_PIN = 13;
constexpr uint8_t LEFT_ULTRASONIC_ECHO_PIN = A0;
constexpr uint8_t RIGHT_ULTRASONIC_ECHO_PIN = A1;
constexpr uint8_t LEFT_ACTIVE_IR_PIN = 10; //swapped with kill switch
constexpr uint8_t RIGHT_ACTIVE_IR_PIN = 3;

// Explicit declarations also allow direct C++ compilation of this .ino.
void stopMoving();
void turnFanOff();
void setMotors(int leftSpeed, int rightSpeed);
bool runDemo();

// Latest uploaded kill-switch behavior.

constexpr unsigned long DEBOUNCE_MS = 50UL;

enum class SafetyState : uint8_t {
    STARTUP,       // Initial 10-second power-on delay
    RUNNING,       // Normal mission is allowed to use motors and fan
    WAIT_SECOND,   // Killed; waiting up to 1 second for a second press
    OFF,           // Remain killed until a new double press
    REPOSITION,    // 60 seconds, or 2 seconds after the third press
    READY,         // Timer expired; awaiting a valid vision position
    ERROR          // Invalid marker or vision initialization failure
};

volatile SafetyState state = SafetyState::STARTUP;
volatile unsigned long firstPressTime = 0;
volatile unsigned long timerStart = 0;
volatile unsigned long timerLength = 0;
volatile unsigned long lastPressTime = 0;
volatile bool havePressed = false;
volatile bool thirdPressed = false;

// A falling edge is a press because the button uses INPUT_PULLUP.
// Cut power here, not in loop(), so long driving loops can be interrupted.
void onButtonPress() {
    const unsigned long now = millis();

    if (havePressed && now - lastPressTime < DEBOUNCE_MS)
        return;

    havePressed = true;
    lastPressTime = now;

    if (state == SafetyState::ERROR) {
        digitalWrite(POWER_RELAY_PIN, RELAY_OFF);
        return;
    }

    if (state == SafetyState::WAIT_SECOND &&
        now - firstPressTime <= DOUBLE_PRESS_MS) {
        // Second press: prepare to restart, but do not move yet.
        state = SafetyState::REPOSITION;
        timerStart = now;
        timerLength = REPOSITION_MS;
        thirdPressed = false;
        return;
    }

    if (state == SafetyState::REPOSITION &&
        now - timerStart < timerLength && !thirdPressed) {
        // Third press: replace the 60-second timer with 2 seconds.
        thirdPressed = true;
        timerStart = now;
        timerLength = EARLY_START_MS;
        return;
    }

    // First press, late second press, fourth press, or any press while
    // running: kill actuator power immediately and start a new 1s window.
    digitalWrite(POWER_RELAY_PIN, RELAY_OFF);
    firstPressTime = now;
    state = SafetyState::WAIT_SECOND;
}

void setupSafety() {
    // Set the relay's OFF level before its pin becomes an output.
    digitalWrite(POWER_RELAY_PIN, RELAY_OFF);
    pinMode(POWER_RELAY_PIN, OUTPUT);
    pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);

    timerStart = millis();
    timerLength = STARTUP_DELAY_MS;
    state = SafetyState::STARTUP;
    attachInterrupt(digitalPinToInterrupt(KILL_SWITCH_PIN), onButtonPress, FALLING);
}

void updateSafety() {
    noInterrupts();
    const unsigned long now = millis();
    if ((state == SafetyState::STARTUP || state == SafetyState::REPOSITION) &&
        now - timerStart >= timerLength) {
        state = SafetyState::READY;
    } else if (state == SafetyState::WAIT_SECOND && now - firstPressTime > DOUBLE_PRESS_MS) {
        state = SafetyState::OFF;
    }
    interrupts();
}

bool missionStartReady() {
    return state == SafetyState::READY;
}

void enableIdlePower() {
    // Main calls this ONLY after commanding motors and fan OFF. This is
    // intentionally not done in the interrupt: motor outputs may be stale
    // until an interrupted route has returned to loop().
    noInterrupts();
    if (state == SafetyState::REPOSITION || state == SafetyState::READY) {
        digitalWrite(POWER_RELAY_PIN, RELAY_ON);
    }
    interrupts();
}

void enableRobotPower() {
    noInterrupts();
    if (state == SafetyState::READY) {
        digitalWrite(POWER_RELAY_PIN, RELAY_ON);
        state = SafetyState::RUNNING;
    }
    interrupts();
}

bool robotEnabled() {
    return state == SafetyState::RUNNING;
}

void disableRobotPower() {
    noInterrupts();
    // Do not erase button presses that happened during the old route.
    if (state == SafetyState::RUNNING) {
        digitalWrite(POWER_RELAY_PIN, RELAY_OFF);
        state = SafetyState::OFF;
    }
    interrupts();
}

void inhibitRobotPower() {
    noInterrupts();
    digitalWrite(POWER_RELAY_PIN, RELAY_OFF);
    state = SafetyState::ERROR;
    interrupts();
}

// Original H-bridge polarity, PWM speeds, fan output, and motor/fan exclusion.
void setupActuators() {
    digitalWrite(FAN_PIN, LOW);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(LEFT_MOTORS_PWM_PIN, OUTPUT);
    pinMode(LEFT_MOTORS_IN1_PIN, OUTPUT);
    pinMode(LEFT_MOTORS_IN2_PIN, OUTPUT);
    pinMode(RIGHT_MOTORS_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTORS_IN1_PIN, OUTPUT);
    pinMode(RIGHT_MOTORS_IN2_PIN, OUTPUT);
    stopMoving();
}

void turnFanOff() { digitalWrite(FAN_PIN, LOW); }

void setMotors(int leftSpeed, int rightSpeed) {
    if (!robotEnabled()) leftSpeed = rightSpeed = 0;
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    if (leftSpeed != 0 || rightSpeed != 0) turnFanOff();
    digitalWrite(LEFT_MOTORS_IN1_PIN, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(LEFT_MOTORS_IN2_PIN, leftSpeed < 0 ? HIGH : LOW);
    analogWrite(LEFT_MOTORS_PWM_PIN, abs(leftSpeed));
    digitalWrite(RIGHT_MOTORS_IN1_PIN, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(RIGHT_MOTORS_IN2_PIN, rightSpeed < 0 ? HIGH : LOW);
    analogWrite(RIGHT_MOTORS_PWM_PIN, abs(rightSpeed));
}

void stopMoving() { setMotors(0, 0); }

void turnFanOn() {
    stopMoving();
    if (robotEnabled()) digitalWrite(FAN_PIN, HIGH);
}

bool waitEnabled(unsigned long duration) {
    const unsigned long start = millis();
    while (robotEnabled() && millis() - start < duration) delay(5);
    return robotEnabled();
}

void setupObstacleSensors() {
    digitalWrite(COMMON_ULTRASONIC_TRIG_PIN, LOW);
    pinMode(COMMON_ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(LEFT_ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(RIGHT_ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(LEFT_ACTIVE_IR_PIN, INPUT);
    pinMode(RIGHT_ACTIVE_IR_PIN, INPUT);
}

// Uploaded shared-trigger / simultaneous-echo implementation. Expose each echo
// for calibration; getFrontDistance still returns the nearer valid reading.
void readFrontDistances(int &leftCm, int &rightCm) {
    static unsigned long lastTrigger = 0;
    const unsigned long elapsed = millis() - lastTrigger;
    if (elapsed < 60) delay(60 - elapsed);
    lastTrigger = millis();
    digitalWrite(COMMON_ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(COMMON_ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(COMMON_ULTRASONIC_TRIG_PIN, LOW);
    const unsigned long startTime = micros();
    unsigned long leftStart = 0, rightStart = 0, leftDuration = 0, rightDuration = 0;
    uint8_t leftState = 0, rightState = 0;
    while ((leftState != 2 || rightState != 2) && micros() - startTime < 25000UL) {
        const unsigned long now = micros();
        if (leftState == 0 && digitalRead(LEFT_ULTRASONIC_ECHO_PIN)) {
            leftStart = now; leftState = 1;
        } else if (leftState == 1 && !digitalRead(LEFT_ULTRASONIC_ECHO_PIN)) {
            leftDuration = now - leftStart; leftState = 2;
        }
        if (rightState == 0 && digitalRead(RIGHT_ULTRASONIC_ECHO_PIN)) {
            rightStart = now; rightState = 1;
        } else if (rightState == 1 && !digitalRead(RIGHT_ULTRASONIC_ECHO_PIN)) {
            rightDuration = now - rightStart; rightState = 2;
        }
    }
    leftCm = leftState == 2 ? (leftDuration * 343UL + 10000UL) / 20000UL : -1;
    rightCm = rightState == 2 ? (rightDuration * 343UL + 10000UL) / 20000UL : -1;
}

int getFrontDistance() {
    int leftCm, rightCm;
    readFrontDistances(leftCm, rightCm);
    if (leftCm < 0) return rightCm;
    if (rightCm < 0) return leftCm;
    return min(leftCm, rightCm);
}

bool runDemo() {
    Serial.println("left_cm,right_cm,nearest_cm,left_IR_detected,right_IR_detected");
    while (robotEnabled()) {
        int leftCm, rightCm;
        readFrontDistances(leftCm, rightCm);
        if (!robotEnabled()) return false;
        const int nearest = leftCm < 0 ? rightCm : rightCm < 0 ? leftCm : min(leftCm, rightCm);
        Serial.print(leftCm); Serial.print(','); Serial.print(rightCm); Serial.print(',');
        Serial.print(nearest); Serial.print(',');
        Serial.print(digitalRead(LEFT_ACTIVE_IR_PIN) == LOW); Serial.print(',');
        Serial.println(digitalRead(RIGHT_ACTIVE_IR_PIN) == LOW);
        if (!waitEnabled(250)) return false;
    }
    return false;
}

void setup() {
    setupSafety(); // Relay OFF and button interrupt active before any other work.
    setupActuators();
    Serial.begin(9600);
    setupObstacleSensors();
}

void loop() {
    updateSafety();
    if (!robotEnabled()) {
        // Clear interrupted actuator commands BEFORE re-energizing the relay.
        stopMoving();
        turnFanOff();
        enableIdlePower();
    }
    if (!missionStartReady()) { delay(10); return; }
    enableRobotPower();
    if (!robotEnabled()) return;
    const bool success = runDemo();
    stopMoving();
    turnFanOff();
    const bool completedWithoutKill = robotEnabled();
    disableRobotPower(); // Preserve double/third presses received during the demo.
    if (completedWithoutKill)
        Serial.println(success ? "Demo complete. Double press to repeat." : "Demo stopped before completion.");
}
