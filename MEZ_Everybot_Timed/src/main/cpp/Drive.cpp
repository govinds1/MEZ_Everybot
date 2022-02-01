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
}

void Drive::Init() {

}

void Drive::Periodic() {

}

void Drive::TankDrive(double left, double right) {
    m_drivebase->TankDrive(left * MOTOR_SPEEDS::DRIVE_MAX_SPEED, right * MOTOR_SPEEDS::DRIVE_MAX_SPEED);
}

void Drive::ArcadeDrive(double speed, double rot) {
    m_drivebase->ArcadeDrive(speed * MOTOR_SPEEDS::DRIVE_MAX_SPEED, rot * MOTOR_SPEEDS::DRIVE_MAX_SPEED);
}