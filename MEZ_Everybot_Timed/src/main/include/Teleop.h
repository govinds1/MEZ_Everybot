#pragma once

#include "Drive.h"
#include "Intake.h"
#include <frc/XboxController.h>

class Teleop {
    public:
    Teleop(Drive* drive, Intake* intake);

    frc::XboxController* m_controller1;
    Drive* m_drive;
    Intake* m_intake;

    void Init();
    void Periodic();
    void SetDriveMult();

    private:
    double driveMult;
};