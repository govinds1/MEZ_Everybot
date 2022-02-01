#pragma once

#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

class Intake {
    public:
    Intake();

    rev::CANSparkMax* m_armMotor;
    WPI_VictorSPX* m_intakeMotor;
    
    void Init();
    void Periodic();

    void IntakeDump();
    void IntakeGrab();
    void IntakeStop();

    void ArmUp();
    void ArmDown();
    void ArmHold();

    private:

    const double armUpPos = 0; // CHANGE FROM TESTING
    const double armDownPos = 0; // CHANGE FROM TESTING
    
};