#pragma once

#include "rev/CANSparkMax.h"
#include "rev/SparkMaxRelativeEncoder.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

class Drive {
    public:
    Drive();

    // MEZ Everybot uses NEOs on the drivetrain
    rev::CANSparkMax m_leftFrontMotor{CAN_IDs::LEFTFRONT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_leftRearMotor{CAN_IDs::LEFTREAR_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_rightFrontMotor{CAN_IDs::RIGHTFRONT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_rightRearMotor{CAN_IDs::RIGHTREAR_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    // rev::CANSparkMax m_leftThirdMotor{CAN_IDs::LEFTTHIRD_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    // rev::CANSparkMax m_rightThirdMotor{CAN_IDs::RIGHTTHIRD_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

    frc::DifferentialDrive* m_drivebase;

    double leftZeroPos;
    double rightZeroPos;
    double leftSetpoint;
    double rightSetpoint;
    rev::SparkMaxRelativeEncoder m_leftEnc = m_leftFrontMotor.GetEncoder();
    rev::SparkMaxRelativeEncoder m_rightEnc = m_rightFrontMotor.GetEncoder();

    void Init();
    void Periodic();
    void TankDrive(double left, double right);
    void ArcadeDrive(double speed, double rot);
    double GetPosition();
    double GetLeftPosition();
    double GetRightPosition();
    double GetAngle();
    void ResetPosition();
    void ConfigPID(rev::CANSparkMax* motor, double kP, double kI, double kD);
    void SetDistance(double leftDist, double rightDist);
    bool AtSetpoint();

    const double kDrivePosThreshold = 0.1;
    const double kEncConvFactor = 42*M_PI*0.5/5.95;
    const double kDriveBaseWidth = 1.75;
};