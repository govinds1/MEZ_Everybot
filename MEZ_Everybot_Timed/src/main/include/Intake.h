#pragma once

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

class Intake {
    public:
    Intake();

    rev::CANSparkMax m_armMotor{CAN_IDs::ARM_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    WPI_VictorSPX m_intakeMotor{CAN_IDs::INTAKE_MOTOR};
    rev::SparkMaxRelativeEncoder m_armEnc = m_armMotor.GetEncoder();
    
    void Init();
    void Periodic();

    void IntakeDump();
    void IntakeGrab();
    void IntakeStop();

    void ArmUp();
    void ArmDown();
    void ArmHold();
    void ArmConfigPID(double kP, double kI, double kD);
    double GetArmPosition();
    void SetArmPosition(double setpoint);

    private:
    bool holding = false;
    double holdPos = 0;

    const double armUpPos = 0; // CHANGE FROM TESTING - encoder count when arm is up
    const double armDownPos = 0; // CHANGE FROM TESTING - encoder count when arm is down
    
};