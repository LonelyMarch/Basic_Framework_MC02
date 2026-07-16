#include "motor_task.h"
#include "LK9025.h"
#include "HT04.h"
#include "htm_rs485.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ddt_motor.h"

void MotorControlTask(void)
{
    DJIMotorControl();
    LKMotorControl();
    HTMotorControl();
    HTMRS485Control();
    DDTMotorControl();
    DMMotorControl();
}
