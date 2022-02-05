#include "Teleop.h"

// ALL Teleop control of subsystems should be handled here

Teleop::Teleop(Drive* drive, Intake* intake): m_drive(drive), m_intake(intake)  {
    m_controller1 = new frc::XboxController(CONTROLLER_CONFIG::CONTROLLER1_PORT);
}

void Teleop::Init() {
    m_drive->Init();
    m_intake->Init();
}

void Teleop::Periodic() {
    m_drive->Periodic();
    m_intake->Periodic();

    // drive
    m_drive->TankDrive(m_controller1->GetLeftY(), m_controller1->GetRightY());

    // intake motor
    if (m_controller1->GetAButton()) {
        m_intake->IntakeGrab();
    } else if (m_controller1->GetBButton()) {
        m_intake->IntakeDump();
    } else {
        m_intake->IntakeStop();
    }

    // arm motor
    if (m_controller1->GetLeftBumper()) {
        m_intake->ArmUp();
    } else if (m_controller1->GetRightBumper()) {
        m_intake->ArmDown();
    } else {
        m_intake->ArmHold();
    }
}