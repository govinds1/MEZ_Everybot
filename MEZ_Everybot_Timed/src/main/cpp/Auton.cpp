#include "Auton.h"

Auton::Auton(Drive* drive, Intake* intake): m_drive(drive), m_intake(intake) {
    // for (std::string option : kAutoNames) {
    //     if (option == "Idle") {
    //         m_chooser.SetDefaultOption(option, option);
    //     } else {
    //         m_chooser.AddOption(option, option);
    //     }
    // }
    // frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    frc::SmartDashboard::PutStringArray("Auto List", kAutoNames);
}

void Auton::Init() {
    m_drive->Init();
    m_intake->Init();

    // Get selected Auton, with default if none selected
    m_autonSelectedString = frc::SmartDashboard::GetString("Auto Selector", "Idle");
    fmt::print("Auton selected: {}\n", m_autonSelectedString);

    m_autonSelectedNumber = 0;
    for (std::string option : kAutoNames) {
        if (m_autonSelectedString == option) {
            break;
        } else {
            m_autonSelectedNumber++;
        }
    }
    stateStartPos = m_drive->GetPosition();
    stateStartAngle = m_drive->GetAngle();
    currentState = 0;
    stateStartTime = GetTime();
}

void Auton::Periodic() {
    m_drive->Periodic();
    m_intake->Periodic();
    switch(m_autonSelectedNumber) {
        case 0: 
            Idle_Auton();
            break;
        case 1:
            Taxi_Auton_DistCheck();
            break;
        case 2:
            DumpAndTaxi_Auton();
            break;
        case 3:
            GrabAndDumpTwo();
            break;
        default:
            m_autonSelectedNumber = 0;
            Idle_Auton();
            break;
    }
}

double Auton::GetTime() {
    return (double)timer.GetFPGATimestamp();
}

void Auton::GoToNextState() {
    m_drive->ResetPosition();
    stateStartTime = GetTime();
    stateStartPos = m_drive->GetPosition();
    stateStartAngle = m_drive->GetAngle();
    currentState++;
}

void Auton::StopSubsystems(bool drive, bool arm, bool intake) {
    if (drive) {
        m_drive->ArcadeDrive(0, 0);
    }
    if (arm) {
        m_intake->ArmHold();
    }
    if (intake) {
        m_intake->IntakeStop();
    }
}

void Auton::Idle_Auton() {
    StopSubsystems(true, true, true);
}


// three implementations of a simple taxi auton
// one time based, one with PID control, one with simple driving and encoder checks

void Auton::Taxi_Auton_Timed() {
    switch(currentState) {
        case 0: // Drive backwards
            m_drive->ArcadeDrive(-0.5, 0);
            StopSubsystems(false, true, true);
            if (GetTime() - stateStartTime >= 5.0) {
                GoToNextState();
            }
            break;
        case 2: // Stop all motors
            StopSubsystems(true, true, true);
            break;
        default:
            StopSubsystems(true, true, true);
            break;
    }
}

void Auton::Taxi_Auton_PID() {
    switch(currentState) {
        case 0: // Set drive setpoint
            m_drive->SetDistance(-5, -5);
            GoToNextState();
            break;
        case 1: // Check if AtSetpoint
            if (m_drive->AtSetpoint()) {
                GoToNextState();
            }
            StopSubsystems(false, true, true);
            break;
        case 2: // Stop all motors
            StopSubsystems(true, true, true);
            break;
        default:
            StopSubsystems(true, true, true);
            break;
    }
}

void Auton::Taxi_Auton_DistCheck() {
    switch(currentState) {
        case 0: // Drive backwards
            m_drive->ArcadeDrive(-0.5, 0);
            StopSubsystems(false, true, true);
            if (std::abs(m_drive->GetPosition() - stateStartPos) >= 5.0) {
                GoToNextState();
            }
            break;
        case 2: // Stop all motors
            StopSubsystems(true, true, true);
            break;
        default:
            StopSubsystems(true, true, true);
            break;
    }
}

void Auton::DumpAndTaxi_Auton() {
    switch(currentState) {
        case 0: // Dump ball
            m_intake->IntakeDump();
            StopSubsystems(true, true, false);
            if (GetTime() - stateStartTime >= 3) {
                GoToNextState();
            }
            break;
        case 1: // Drive backwards
            m_drive->ArcadeDrive(-0.5, 0);
            StopSubsystems(false, true, true);

            // use distance checks with the drive encoders, but using Timer for now
            if (GetTime() - stateStartTime >= 5.0) {
                GoToNextState();
            }
            break;
        case 2: // Stop all motors
            StopSubsystems(true, true, true);
            break;
        default:
            StopSubsystems(true, true, true);
            break;
    }
}

void Auton::GrabAndDumpTwo() {
    // START FACING BALL TO PICK UP
    switch (currentState)
    {
        case 0: // Move arm down and run intake while driving forwards
            m_drive->ArcadeDrive(0.5, 0);
            m_intake->ArmDown();
            m_intake->IntakeGrab();
            StopSubsystems(false, false, false);
            if (m_drive->GetPosition() - stateStartPos >= 5) {
                GoToNextState();
            }
            break;
        case 1: // Turn 180 degrees
            m_drive->ArcadeDrive(0, -0.5);
            StopSubsystems(false, true, true);
            if (std::abs(m_drive->GetAngle() - stateStartAngle) <= 2) {
                GoToNextState();
            }
            break;
        case 2: // Drive forwards and arm up
            m_drive->ArcadeDrive(0.5, 0);
            m_intake->ArmUp();
            StopSubsystems(false, false, true);
            if (m_drive->GetPosition() - stateStartPos >= 7) {
                GoToNextState();
            }
        case 3:
            m_intake->IntakeDump();
            StopSubsystems(true, true, false);
            if (GetTime() - stateStartTime >= 5) {
                GoToNextState();
            }
        case 4:
            StopSubsystems(true, true, true);
            break;
        default:
            StopSubsystems(true, true, true);
            break;
    }
}