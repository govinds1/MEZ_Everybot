#pragma once

#include <string>
#include <vector>
#include "Drive.h"
#include "Intake.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

class Auton {
    public:
    Auton(Drive* drive, Intake* intake);

    void Init();
    void Periodic();

    // Auton functions
    void Idle_Auton();
    void Taxi_Auton();
    void DumpAndTaxi_Auton();
    // add more

    Drive* m_drive;
    Intake* m_intake;

    double GetTime();
    void GoToNextState();
    void StopSubsystems(bool drive, bool arm, bool intake);

    private:
    frc::SendableChooser<std::string> m_chooser;
    std::string m_autonSelectedString;
    int m_autonSelectedNumber;
    int currentState;
    double stateStart;
    frc::Timer timer;

    // Update this vector when you add more routines
    const std::vector<std::string> kAutoNames{"Idle", "Taxi", "DumpAndTaxi"};
};