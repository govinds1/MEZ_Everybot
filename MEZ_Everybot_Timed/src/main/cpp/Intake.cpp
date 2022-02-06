#include "Intake.h"

Intake::Intake() {
    m_armMotor = new rev::CANSparkMax(CAN_IDs::ARM_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_intakeMotor = new WPI_VictorSPX(CAN_IDs::INTAKE_MOTOR);

    m_armMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_armMotor->SetInverted(false);
    m_armMotor->SetOpenLoopRampRate(0.2);
    m_armMotor->SetClosedLoopRampRate(0.2);

    m_intakeMotor->SetInverted(false);

    ArmConfigPID(0.0, 0.0, 0.0);
    // set soft limits for arm motor to armUpPos and armDownPos
    //m_armMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armUpPos);
    //m_armMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armDownPos);

    m_armMotor->GetEncoder().SetPositionConversionFactor(1.0);

    frc::SmartDashboard::PutNumber("Subsystems/Arm/Position", GetArmPosition());
    frc::SmartDashboard::PutBoolean("Subsystems/Arm/Holding", holding);
    frc::SmartDashboard::PutBoolean("Subsystems/Arm/HoldPos", holdPos);
}

void Intake::Init() {
    holding = false;
    IntakeStop();
    ArmHold();
}

void Intake::Periodic() {
    // push current encoder value to SmartDashboard
    frc::SmartDashboard::PutNumber("Subsystems/Arm/Position", GetArmPosition());
    frc::SmartDashboard::PutBoolean("Subsystems/Arm/Holding", holding);
    frc::SmartDashboard::PutBoolean("Subsystems/Arm/HoldPos", holdPos);
    
}

void Intake::IntakeDump() {
    m_intakeMotor->Set(-MOTOR_SPEEDS::INTAKE_SPEED);
}

void Intake::IntakeGrab() {
    m_intakeMotor->Set(MOTOR_SPEEDS::INTAKE_SPEED);
}

void Intake::IntakeStop() {
    m_intakeMotor->Set(0);
}

void Intake::ArmUp() {
    // use setpoints if possible, otherwise just Set(arm speed)
    // set position setpoint to be some predetermined encoder value for up
    //m_armEncoder.SetPosition(armUpPos);
    holding = false;

    m_armMotor->Set(MOTOR_SPEEDS::ARM_SPEED);

}

void Intake::ArmDown() {
    // use setpoints if possible, otherwise just Set(-arm speed)
    // set position setpoint to be some predetermined encoder value for down
    //m_armEncoder.SetPosition(armDownPos);
    holding = false;

    m_armMotor->Set(-MOTOR_SPEEDS::ARM_SPEED);
}

void Intake::ArmHold() {
    // use setpoints if possible, otherwise just Set(0)
    // for hold setpoint, just get current encoder value and set that as position setpoint
    // if (!holding) {
    //     holdPos = GetArmPosition();
    //     holding = true;
    // }
    // SetArmPosition(holdPos);

    m_armMotor->Set(0);
}

void Intake::ArmConfigPID(double kP, double kI, double kD) {
    auto armPIDController = m_armMotor->GetPIDController();
    armPIDController.SetP(kP);
    armPIDController.SetI(kI);
    armPIDController.SetD(kD);
}

double Intake::GetArmPosition() {
    // do any conversions if needed
    return m_armMotor->GetEncoder().GetPosition();
}

void Intake::SetArmPosition(double setpoint) {
    // do any conversions if needed
    m_armMotor->GetEncoder().SetPosition(setpoint);
}