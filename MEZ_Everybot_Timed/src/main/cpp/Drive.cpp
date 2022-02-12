#include "Drive.h"
#include <math.h>

Drive::Drive() {
    m_leftRearMotor.Follow(m_leftFrontMotor);
    m_rightRearMotor.Follow(m_rightFrontMotor);
    //m_leftThirdMotor.Follow(m_leftFrontMotor);
    //m_rightThirdMotor.Follow(m_rightFrontMotor);

    m_leftFrontMotor.SetInverted(false);
    m_leftRearMotor.SetInverted(false);
    m_rightFrontMotor.SetInverted(false);
    m_rightRearMotor.SetInverted(false);
    //m_leftThirdMotor.SetInverted(false);
    //m_rightThirdMotor.SetInverted(false);

    m_leftFrontMotor.SetOpenLoopRampRate(0.2);
    m_leftRearMotor.SetOpenLoopRampRate(0.2);
    m_rightFrontMotor.SetOpenLoopRampRate(0.2);
    m_rightRearMotor.SetOpenLoopRampRate(0.2);

    m_drivebase = new frc::DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);

    // use counts per revolution and the wheel circumference to get conversion factor from encoder units to feet
    m_leftEnc.SetPositionConversionFactor(kEncConvFactor);
    m_rightEnc.SetPositionConversionFactor(kEncConvFactor);

    ConfigPID(&m_leftFrontMotor, 0.0, 0.0, 0.0);
    ConfigPID(&m_rightFrontMotor, 0.0, 0.0, 0.0);

    ResetPosition();

    frc::SmartDashboard::PutNumber("Subsystems/Drive/Left/Position", GetLeftPosition());
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Right/Position", GetRightPosition());
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Position", GetPosition());
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Angle", GetAngle());
}

void Drive::Init() {
    ResetPosition();
    ArcadeDrive(0, 0);
}

void Drive::Periodic() {
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Left/Position", GetLeftPosition());
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Right/Position", GetRightPosition());
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Position", GetPosition());
    frc::SmartDashboard::PutNumber("Subsystems/Drive/Angle", GetAngle());
}

void Drive::TankDrive(double left, double right) {
    m_drivebase->TankDrive(left * MOTOR_SPEEDS::DRIVE_MAX_SPEED, right * MOTOR_SPEEDS::DRIVE_MAX_SPEED);
}

void Drive::ArcadeDrive(double speed, double rot) {
    m_drivebase->ArcadeDrive(speed * MOTOR_SPEEDS::DRIVE_MAX_SPEED, rot * MOTOR_SPEEDS::DRIVE_MAX_SPEED);
}

double Drive::GetPosition() {
    // average of the front encoders
    return (GetLeftPosition() + GetRightPosition())/2.0;
}

double Drive::GetLeftPosition() {
    return m_leftEnc.GetPosition() - leftZeroPos;
}

double Drive::GetRightPosition() {
    return m_rightEnc.GetPosition() - rightZeroPos;
}

double Drive::GetAngle() {
    return (((GetRightPosition() - GetLeftPosition()) / kDriveBaseWidth)/M_PI)*180.0;
}

void Drive::ResetPosition() {
    leftZeroPos = m_leftEnc.GetPosition();
    rightZeroPos = m_rightEnc.GetPosition();
    leftSetpoint = leftZeroPos;
    rightSetpoint = rightZeroPos;
}

void Drive::ConfigPID(rev::CANSparkMax* motor, double kP, double kI, double kD) {
    auto motorPIDController = motor->GetPIDController();
    motorPIDController.SetP(kP);
    motorPIDController.SetI(kI);
    motorPIDController.SetD(kD);
}

void Drive::SetDistance(double leftDist, double rightDist) {
    // Make sure this is only called once, at the beginning when you set the setpoint
    // If it is called while the robot should be driving to the setpoint, the call to GetPosition will continually change the setpoint

    // might have to multiply by a conversion factor
    leftSetpoint = leftDist + GetLeftPosition();
    rightSetpoint = rightDist + GetRightPosition();
    m_leftEnc.SetPosition(leftSetpoint);
    m_rightEnc.SetPosition(rightSetpoint);
}

bool Drive::AtSetpoint() {
    if (std::abs(GetLeftPosition() - leftSetpoint) > kDrivePosThreshold) {
        return false;
    } else if (std::abs(GetRightPosition() - rightSetpoint) > kDrivePosThreshold) {
        return false;
    }
    return true;
}