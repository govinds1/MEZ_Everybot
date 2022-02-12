#pragma once

#include <frc/XboxController.h>

namespace CAN_IDs {
    static constexpr int LEFTFRONT_MOTOR = 1;
    static constexpr int LEFTREAR_MOTOR = 2;
    static constexpr int RIGHTFRONT_MOTOR = 3;
    static constexpr int RIGHTREAR_MOTOR = 4;
    static constexpr int ARM_MOTOR = 5;
    static constexpr int INTAKE_MOTOR = 6;
    //static constexpr int LEFTTHIRD_MOTOR = 7;
    //static constexpr int RIGHTTHIRD_MOTOR = 8;
}

namespace MOTOR_SPEEDS {
    static constexpr double DRIVE_MAX_SPEED = 0.9;
    static constexpr double INTAKE_SPEED = 0.9;
    static constexpr double ARM_SPEED = 0.3; // maybe have one for up and one for down?
}

namespace CONTROLLER_CONFIG {
    static constexpr int CONTROLLER1_PORT = 0;
}