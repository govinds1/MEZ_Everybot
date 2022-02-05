#pragma once

#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

class Drive {
    public:
    Drive();

    rev::CANSparkMax* m_leftFrontMotor;
    rev::CANSparkMax* m_leftRearMotor;
    rev::CANSparkMax* m_rightFrontMotor;
    rev::CANSparkMax* m_rightRearMotor;

    frc::DifferentialDrive* m_drivebase;

    double leftZeroPos;
    double rightZeroPos;

    void Init();
    void Periodic();
    void TankDrive(double left, double right);
    void ArcadeDrive(double speed, double rot);
    double GetPosition();
    double GetLeftPosition();
    double GetRightPosition();
    void ResetPosition();
};