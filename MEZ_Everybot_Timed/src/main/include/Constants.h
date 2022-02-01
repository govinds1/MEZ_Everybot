#pragma once

namespace CAN_IDs {
    static constexpr int LEFTFRONT_MOTOR = 0;
    static constexpr int LEFTREAR_MOTOR = 1;
    static constexpr int RIGHTFRONT_MOTOR = 2;
    static constexpr int RIGHTREAR_MOTOR = 3;
    static constexpr int ARM_MOTOR = 4;
    static constexpr int INTAKE_MOTOR = 5;
}

namespace MOTOR_SPEEDS {
    static constexpr double DRIVE_MAX_SPEED = 0.9;
    static constexpr double INTAKE_SPEED = 0.9;
    static constexpr double ARM_SPEED = 0.5; // maybe have one for up and one for down?
}

namespace CONTROLLER_CONFIG {
    static constexpr int CONTROLLER1_PORT = 0;
    static constexpr int CONTROLLER2_PORT = 1;
    // add buttons?
}