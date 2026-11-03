#pragma once
#include <Arduino.h>

void navigateTo(int x, int y);
void navigateTo(int x, int y, int speed);

int getCurrentX();
int getCurrentY();
int getTheta();