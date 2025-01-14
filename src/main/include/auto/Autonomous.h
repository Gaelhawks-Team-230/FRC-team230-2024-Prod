#pragma once

#include <optional>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc/Errors.h>

#include "auto/modes/AutoMode.h"
#include "auto/modes/DoNothing.h"
#include "auto/modes/Leave.h"
// #include "auto/modes/TestPath.h"
#include "auto/modes/ThreeNoteSourceSide.h"
// #include "auto/modes/ThreeNoteBetween.h"
// #include "auto/modes/FourNoteAmpToSource.h"
// #include "auto/modes/FourNoteCenter.h"
#include "auto/modes/FourNoteSourceToAmp.h"
#include "auto/modes/ThreeNoteAmpSide.h"
// #include "auto/modes/The195Special.h"
#include "auto/modes/FiveNoteAmpSide.h"
#include "auto/modes/ThreeNoteSourceSide2Center.h"
#include "auto/modes/FourNoteTRUECenter.h"
#include "auto/modes/FourNoteCloseFarClose.h"
#include "auto/modes/ThreeNoteCenterToSource.h"
#include "auto/modes/FourNoteSourceToAmpNEW.h"

#include "RobotStateMachine.h"

#include "auto/modes/AutoMode.h"
#include "Constants.h"

// format: "n Note Desc [Start Pos] {Note #s}"
const std::string kAutoNameDefault = "Do Nothing";
const std::string kAutoPathA = "Leave";
// const std::string kAutoPathB = "Test Path";
// const std::string kAutoPathC = "3 Note Between";
const std::string kAutoPathD = "3 Note Source Side"; 
// const std::string kAutoPathE = "4 Note Amp To Source Side";
const std::string kAutoPathF = "4 Note Source To Amp Side";
const std::string kAutoPathG = "4 Note Amp Side & Centerline";
// const std::string kAutoPathH = "4 Note Center";
const std::string kAutoPathI = "3 Note Amp Side"; 
// const std::string kAutoPathJ = "The 195 Special";
const std::string kaAutoPathK = "3 Note Source Side 2Center"; 
const std::string kAutoPathL = "3.5 TRUE Center"; 
const std::string kAutoPathM = "4 Note Center Close Far Close"; 
const std::string kAutoPathN = "3 Note Center To Source Up-Close"; 
const std::string kAutoPathO = "4 Note Source To Amp NEW !!";
// "4 Note [Center] {1, 3, 7, 4}"

class Autonomous
{
public:
    static Autonomous *GetInstance();

    void UpdateAutoSelection();
    void Reset();
    void RunAuto();

    /**
     * @brief Get the Auto Initial Robot Heading object
     *
     * @return std::optional<double> heading in degrees
     */
    std::optional<double> GetAutoInitialRobotHeading()
    {
        if (m_autoMode)
            return m_autoMode->GetInitialRobotHeading();
        else
            return std::nullopt;
    }

private:
    Autonomous();
    AutoMode *GetAutoSelection();

    static Autonomous *m_autonomous;
    RobotStateMachine *m_robotStateMachine;

    frc::SendableChooser<std::string> m_chooser;
    std::string m_autoSelected;
    std::string m_lastSelected;
    AutoMode *m_autoMode;

    frc::Timer m_timer;
    bool m_autoTimerStarted = false;
    bool m_autoModeCompleteMessagePrinted = false;
    bool m_autoModeOvertimeWarningPrinted = false;
};