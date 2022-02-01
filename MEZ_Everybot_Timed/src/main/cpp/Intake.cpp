#include "Intake.h"

Intake::Intake() {
    m_armMotor = new rev::CANSparkMax(CAN_IDs::ARM_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_intakeMotor = new WPI_VictorSPX(CAN_IDs::INTAKE_MOTOR);

    m_armMotor->SetInverted(false);
    m_intakeMotor->SetInverted(false);

    // set soft limits for arm motor to armUpPos and armDownPos
}

void Intake::Init() {
    // possibly reset encoder for arm motor

}

void Intake::Periodic() {
    // push current encoder value to SmartDashboard
    
}

void Intake::Dump() {
    m_intakeMotor->Set(-MOTOR_SPEEDS::INTAKE_SPEED);
}

void Intake::Grab() {
    m_intakeMotor->Set(MOTOR_SPEEDS::INTAKE_SPEED);
}

void Intake::ArmUp() {
    // use setpoints if possible, otherwise just Set(arm speed)
    // set position setpoint to be some predetermined encoder value for up
    m_armMotor->Set(MOTOR_SPEEDS::ARM_SPEED);
}

void Intake::ArmDown() {
    // use setpoints if possible, otherwise just Set(-arm speed)
    // set position setpoint to be some predetermined encoder value for down
    m_armMotor->Set(-MOTOR_SPEEDS::ARM_SPEED);
}

void Intake::ArmHold() {
    // use setpoints if possible, otherwise just Set(0)
    // for hold setpoint, just get current encoder value and set that as position setpoint
    m_armMotor->Set(0);
}