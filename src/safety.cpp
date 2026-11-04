#include "safety.h"

namespace {
    constexpr unsigned long DEBOUNCE_MS = 50;
    constexpr unsigned long DOUBLE_PRESS_MS = 1500;

    enum class SafetyState : uint8_t {
        STARTING,
        RUNNING,
        OFF_WAITING
    };

    volatile SafetyState state = SafetyState::STARTING;

    volatile unsigned long firstPressTime = 0;
    volatile unsigned long lastPressTime = 0;
    volatile bool hasPressed = false;

    void onButtonPress() {
        unsigned long now = millis();

        // Debounce the switch
        if (hasPressed &&
            now - lastPressTime < DEBOUNCE_MS) {
            return;
        }

        hasPressed = true;
        lastPressTime = now;

        // Immediately disconnect actuator power
        digitalWrite(POWER_RELAY_PIN, RELAY_OFF);

        // Second press within 1.5 seconds
        if (state == SafetyState::OFF_WAITING &&
            now - firstPressTime <= DOUBLE_PRESS_MS) {

            NVIC_SystemReset();
        }

        // Otherwise, start a new double-press window
        state = SafetyState::OFF_WAITING;
        firstPressTime = now;
    }
}

void setupSafety() {
    // Ensure relay starts OFF
    digitalWrite(POWER_RELAY_PIN, RELAY_OFF);
    pinMode(POWER_RELAY_PIN, OUTPUT);

    pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(KILL_SWITCH_PIN),
        onButtonPress,
        FALLING
    );
}

void enableRobotPower() {
    noInterrupts();

    // Only enable if no button was pressed during startup
    if (state == SafetyState::STARTING) {
        digitalWrite(POWER_RELAY_PIN, RELAY_ON);
        state = SafetyState::RUNNING;
    }

    interrupts();
}

bool robotEnabled() {
    return state == SafetyState::RUNNING;
}