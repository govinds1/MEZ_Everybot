#include "Teleop.h"

Teleop::Teleop() {
    m_controller1 = new frc::XboxController(CONTROLLER_CONFIG::CONTROLLER1_PORT);
    m_drive = new Drive();
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
    if (m_controller1->GetAButtonPressed()) {
        m_intake->Grab();
    } else if (m_controller1->GetBButtonPressed()) {
        m_intake->Dump();
    }

    // arm motor
    if (m_controller1->GetRightBumper()) {
        m_intake->ArmDown();
    } else if (m_controller1->GetLeftBumper()) {
        m_intake->ArmUp();
    } else {
        m_intake->ArmHold();
    }
}