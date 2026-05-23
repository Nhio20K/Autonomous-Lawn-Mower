#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include <Wire.h>
#include <INA226_WE.h>

extern bool bat_ok;  // true = INA226 init สำเร็จ

void initBattery();
void updateBattery();

#endif
