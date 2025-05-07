#ifndef RTOS_CONTROL_H
#define RTOS_CONTROL_H

#include "rtos_tasks.h"
#include "config.h"

// Control task variables
extern unsigned long lastCommandTime;
extern bool drivingState;
extern unsigned long lastNonNeutralCommand;
extern const unsigned long DRIVING_GRACE_PERIOD;

// Control task function (declared in rtos_tasks.h, implemented in rtos_control.cpp)
// void ControlTask(void *pvParameters);

#endif // RTOS_CONTROL_H