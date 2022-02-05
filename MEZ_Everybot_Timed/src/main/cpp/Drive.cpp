#include "Drive.h"

Drive::Drive() {
    m_leftFrontMotor = new rev::CANSparkMax(CAN_IDs::LEFTFRONT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_leftRearMotor = new rev::CANSparkMax(CAN_IDs::LEFTREAR_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_rightFrontMotor = new rev::CANSparkMax(CAN_IDs::RIGHTFRONT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_rightRearMotor = new rev::CANSparkMax(CAN_IDs::RIGHTREAR_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

    m_leftRearMotor->Follow(*m_leftFrontMotor);
    m_rightRearMotor->Follow(*m_rightFrontMotor);

    m_leftFrontMotor->SetInverted(false);
    m_leftRearMotor->SetInverted(false);
    m_rightFrontMotor->SetInverted(false);
    m_rightRearMotor->SetInverted(false);

    m_leftFrontMotor->SetOpenLoopRampRate(0.2);
    m_leftRearMotor->SetOpenLoopRampRate(0.2);
    m_rightFrontMotor->SetOpenLoopRampRate(0.2);
    m_rightRearMotor->SetOpenLoopRampRate(0.2);

    m_drivebase = new frc::DifferentialDrive(*m_leftFrontMotor, *m_rightFrontMotor);

    // use counts per revolution and the wheel circumference to get conversion factor from encoder units to feet
    m_leftFrontMotor->GetEncoder().SetPositionConversionFactor(1.0);
    m_rightFrontMotor->GetEncoder().SetPositionConversionFactor(1.0);

    ResetPosition();

    frc::SmartDashboard::PutNumber("Subsystems/Drive/Left/Position", GetLeftPosition());
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Right/Position", GetRightPosition());
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Position", GetPosition());
}

void Drive::Init() {
    ResetPosition();
    ArcadeDrive(0, 0);
}

void Drive::Periodic() {
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Left/Position", GetLeftPosition());
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Right/Position", GetRightPosition());
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Position", GetPosition());
}

void Drive::TankDrive(double left, double right) {
    m_drivebase->TankDrive(left * MOTOR_SPEEDS::DRIVE_MAX_SPEED, right * MOTOR_SPEEDS::DRIVE_MAX_SPEED);
}

void Drive::ArcadeDrive(double speed, double rot) {
    m_drivebase->ArcadeDrive(speed * MOTOR_SPEEDS::DRIVE_MAX_SPEED, rot * MOTOR_SPEEDS::DRIVE_MAX_SPEED);
}

double Drive::GetPosition() {
    //some average of the front encoders
    return (GetLeftPosition() + GetRightPosition())/2.0;
}

double Drive::GetLeftPosition() {
    return m_leftFrontMotor->GetEncoder().GetPosition() - leftZeroPos;
}

double Drive::GetRightPosition() {
    return m_rightFrontMotor->GetEncoder().GetPosition() - rightZeroPos;
}

void Drive::ResetPosition() {
    leftZeroPos = m_leftFrontMotor->GetEncoder().GetPosition();
    rightZeroPos = m_rightFrontMotor->GetEncoder().GetPosition();
}