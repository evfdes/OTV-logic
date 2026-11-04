#pragma once
#include <Arduino.h>

void setupNavigation();
bool startedAtBottom();
bool turnToPayload();
bool approachPayload();
bool leavePayload();
bool navigateObstacles();