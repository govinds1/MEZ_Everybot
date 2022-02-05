#include "Auton.h"

Auton::Auton(Drive* drive, Intake* intake): m_drive(drive), m_intake(intake) {
    for (std::string option : kAutoNames) {
        if (option == "Idle") {
            m_chooser.SetDefaultOption(option, option);
        } else {
            m_chooser.AddOption(option, option);
        }
    }
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Auton::Init() {
    m_drive->Init();
    m_intake->Init();

    // comment out chooser line and uncomment dashboard GetString if you use the LabVIEW dashboard to select Auton
    m_autonSelectedString = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString("Auto Selector",
    //     kAutoNameDefault);
    fmt::print("Auton selected: {}\n", m_autonSelectedString);

    m_autonSelectedNumber = 0;
    for (std::string option : kAutoNames) {
        if (m_autonSelectedString == option) {
            break;
        } else {
            m_autonSelectedNumber++;
        }
    }
    currentState = 0;
    stateStart = GetTime();
}

void Auton::Periodic() {
    m_drive->Periodic();
    m_intake->Periodic();
    switch(m_autonSelectedNumber) {
        case 0: 
            Idle_Auton();
            break;
        case 1:
            Taxi_Auton();
            break;
        case 2:
            DumpAndTaxi_Auton();
            break;
        default:
            Idle_Auton();
            break;
    }
}

double Auton::GetTime() {
    return (double)timer.GetFPGATimestamp();
}

void Auton::GoToNextState() {
    m_drive->ResetPosition();
    stateStart = 0.0;
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

void Auton::Taxi_Auton() {
    switch(currentState) {
        case 0:
            m_drive->ArcadeDrive(-0.5, 0);
            StopSubsystems(false, true, true);
            // use distance checks with the drive encoders, but using Timer for now
            if (GetTime() - stateStart >= 5.0) {
                GoToNextState();
            }
            break;
        case 1:
            StopSubsystems(true, true, true);
            break;
        default:
            StopSubsystems(true, true, true);
            break;
    }
}

void Auton::DumpAndTaxi_Auton() {
    switch(currentState) {
        case 0:
            m_intake->IntakeDump();
            StopSubsystems(true, true, false);
            if (GetTime() - stateStart >= 3) {
                GoToNextState();
            }
            break;
        case 1:
            m_drive->ArcadeDrive(-0.5, 0);
            StopSubsystems(false, true, true);

            // use distance checks with the drive encoders, but using Timer for now
            if (GetTime() - stateStart >= 5.0) {
                GoToNextState();
            }
            break;
        case 2:
            StopSubsystems(true, true, true);
            break;
        default:
            StopSubsystems(true, true, true);
            break;
    }
}