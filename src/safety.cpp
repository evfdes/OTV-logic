#include "config.h"
#include "safety.h"

namespace {
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