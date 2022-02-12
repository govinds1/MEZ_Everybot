#include "Teleop.h"

// ALL Teleop control of subsystems should be handled here

Teleop::Teleop(Drive* drive, Intake* intake): m_drive(drive), m_intake(intake)  {
    m_controller1 = new frc::XboxController(CONTROLLER_CONFIG::CONTROLLER1_PORT);
}

void Teleop::Init() {
    driveMult = 1.0;
    m_drive->Init();
    m_intake->Init();
}

void Teleop::Periodic() {
    m_drive->Periodic();
    m_intake->Periodic();

    // drive
    SetDriveMult();
    m_drive->ArcadeDrive(-m_controller1->GetLeftY()*driveMult, m_controller1->GetLeftX()*driveMult);

    // intake motor
    if (m_controller1->GetRightBumper()) {
        m_intake->IntakeGrab();
    } else if (m_controller1->GetLeftBumper()) {
        m_intake->IntakeDump();
    } else {
        m_intake->IntakeStop();
    }

    // arm motor
    // TODO: Remove ArmHold, so the Arm goes to and stays at the up/down position
    // depending on the last trigger pressed
    if (m_controller1->GetRightTriggerAxis() >= 0.05) {
        m_intake->ArmUp();
    } else if (m_controller1->GetLeftTriggerAxis() >= 0.05) {
        m_intake->ArmDown();
    } else {
        m_intake->ArmHold();
    }
}

void Teleop::SetDriveMult() {
    // toggles
    if (m_controller1->GetAButtonPressed()) {
        driveMult = 0.25;
    } else if (m_controller1->GetXButtonPressed()) {
        driveMult = 0.5;
    } else if (m_controller1->GetBButtonPressed()) {
        driveMult = 0.75;
    } else if (m_controller1->GetYButtonPressed()) {
        driveMult = 1.0;
    }

    // holds
    // if (m_controller1->GetAButton()) {
    //     driveMult = 0.25;
    // } else if (m_controller1->GetXButton()) {
    //     driveMult = 0.5;
    // } else if (m_controller1->GetBButton()) {
    //     driveMult = 0.75;
    // } else {
    //     driveMult = 1.0;
    // }
}