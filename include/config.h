#pragma once
#include <Arduino.h>

constexpr int MARKER_ID = 98; // REQUIRED: enter the ID printed on your ArUco marker.
constexpr int ROOM_NUMBER = 1201;
constexpr unsigned long STARTUP_DELAY_MS = 10000;

// Coordinates are in metres: +x right, +y up. Headings are degrees CCW from +x.
constexpr float ARENA_LENGTH_M = 4.0f;
constexpr float ARENA_WIDTH_M = 2.0f;
constexpr float ROBOT_HALF_LENGTH_M = 0.1125f;
constexpr float POSITION_TOLERANCE_M = 0.015f;
constexpr int HEADING_TOLERANCE_DEG = 5;
constexpr unsigned long MOVE_TIMEOUT_MS = 60000;
constexpr unsigned long TURN_TIMEOUT_MS = 15000;

constexpr float TOP_APPROACH_X_M = 0.22f;
constexpr float BOTTOM_APPROACH_X_M = 0.16f;
constexpr float PAYLOAD_CENTER_X_M = 0.50f;
constexpr float LOWER_PAYLOAD_CENTER_Y_M = 0.50f;
constexpr float LOWER_PAYLOAD_CORNER_X_M = 0.37f;
constexpr float LOWER_PAYLOAD_CORNER_Y_M = 0.62f;
constexpr float PAYLOAD_HALF_SIZE_M = 0.1125f;
constexpr int PAYLOAD_STANDOFF_CM = 25;
constexpr float PAYLOAD_EXIT_X_M = 0.80f;

constexpr float MIDDLE_LANE_Y_M = 1.0f;
constexpr float LOWER_LANE_Y_M = 0.40f;
constexpr float OBSTACLE_FRONT_X_M[] = {4.0f / 3.0f, 13.0f / 6.0f};
constexpr float OBSTACLE_DEPTH_M = 1.0f / 6.0f;
constexpr float OBSTACLE_STANDOFF_M = 0.25f;
constexpr float OBSTACLE_REAR_CLEARANCE_M = 0.10f;
constexpr int OBSTACLE_DETECTION_CM = 40;
constexpr float FINISH_X_M = 3.80f;

constexpr int FLAME_THRESHOLD = 800;
constexpr int CENTER_CANDLES = 1; // Four sensors observe the outer candles only.
constexpr unsigned long SEATING_TIME_MS = 5000;
constexpr uint8_t FAN_ATTEMPTS = 3;
constexpr unsigned long FAN_RUN_MS = 10000;
constexpr unsigned long POST_EXTINGUISH_WAIT_MS = 3000;
constexpr unsigned long RUN_DURATION_MS = 300000;
constexpr unsigned long HIGH_SPEED_WINDOW_MS = 30000;
constexpr int FAST_DRIVE_SPEED = 150;
