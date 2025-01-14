#include "auto/Autonomous.h"

Autonomous *Autonomous::GetInstance()
{
    if (m_autonomous == nullptr)
    {
        m_autonomous = new Autonomous();
    }
    return m_autonomous;
}

/**
 * @brief Construct a new Sample:: Sample object
 *
 */
Autonomous::Autonomous()
{   
    m_robotStateMachine = RobotStateMachine::GetInstance();
    
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoPathA, kAutoPathA);
    // m_chooser.AddOption(kAutoPathB, kAutoPathB);
    // m_chooser.AddOption(kAutoPathC, kAutoPathC);
    m_chooser.AddOption(kAutoPathD, kAutoPathD);
    // m_chooser.AddOption(kAutoPathE, kAutoPathE);
    m_chooser.AddOption(kAutoPathF, kAutoPathF);
    m_chooser.AddOption(kAutoPathG, kAutoPathG);
    // m_chooser.AddOption(kAutoPathH, kAutoPathH);
    m_chooser.AddOption(kAutoPathI, kAutoPathI);
    // m_chooser.AddOption(kAutoPathJ, kAutoPathJ);
    m_chooser.AddOption(kaAutoPathK, kaAutoPathK);
    m_chooser.AddOption(kAutoPathL, kAutoPathL);
    m_chooser.AddOption(kAutoPathM, kAutoPathM);
    m_chooser.AddOption(kAutoPathN, kAutoPathN);
    m_chooser.AddOption(kAutoPathO, kAutoPathO);

    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * @brief Updates the auto mode selection if it has changed
 *
 */
void Autonomous::UpdateAutoSelection()
{
    m_autoSelected = m_chooser.GetSelected();

    if (m_autoSelected != m_lastSelected)
    {
        fmt::print("Auto Mode changed to {}\n", m_chooser.GetSelected());
        m_autoMode = GetAutoSelection();
    }
    m_lastSelected = m_autoSelected;
}

/**
 * @brief Create the auto mode based on the selection
 *
 * @return AutoMode*
 */
AutoMode *Autonomous::GetAutoSelection()
{
     if (m_autoSelected == kAutoPathA)
    {
        return new Leave();
    }
    // else if (m_autoSelected == kAutoPathB)
    // {
    //     return new TestPath();
    // }
    // else if (m_autoSelected == kAutoPathC)
    // {
    //     return new ThreeNoteBetween();
    // }
    else if (m_autoSelected == kAutoPathD)
    {
        return new ThreeNoteSourceSide();
    }
    // else if (m_autoSelected == kAutoPathE)
    // {
    //     return new FourNoteAmpToSource();
    // }
    else if (m_autoSelected == kAutoPathF)
    {
        return new FourNoteSourceToAmp();
    }
    else if (m_autoSelected == kAutoPathG)
    {
        return new FiveNoteAmpSide();
    }
    // else if (m_autoSelected == kAutoPathH)
    // {
    //     return new FourNoteCenter();
    // }
    else if (m_autoSelected == kAutoPathI)
    {
        return new ThreeNoteAmpSide();
    }
    // else if (m_autoSelected == kAutoPathJ)
    // {
    //     return new The195Special();
    // }
    else if (m_autoSelected == kaAutoPathK)
    {
        return new ThreeNoteSourceSide2Center();
    }
    else if (m_autoSelected == kAutoPathL)
    {
        return new FourNoteTRUECenter();
    }
    else if (m_autoSelected == kAutoPathM)
    {
        return new FourNoteCloseFarClose();
    }
    else if (m_autoSelected == kAutoPathN)
    {
        return new ThreeNoteCenterToSource();
    }
    else if (m_autoSelected == kAutoPathO)
    {
        return new FourNoteSourceToAmpNEW();
    }
    else
    {
        return new DoNothing();
    }
}

/**
 * @brief Reset the auto mode
 *
 */
void Autonomous::Reset()
{
    m_autoMode = nullptr;
    m_autoSelected = kAutoNameDefault;
    m_lastSelected = kAutoNameDefault;
    m_autoModeCompleteMessagePrinted = false;
    m_autoTimerStarted = false;
    m_autoModeOvertimeWarningPrinted = false;
    m_timer.Stop();
}

/**
 * @brief Run the auto mode
 *
 */
void Autonomous::RunAuto()
{
    if (!m_autoMode)
    {
        FRC_ReportError(frc::err::Error, "Trying to run null auto mode\n");
        return;
    }

    // Start timer only if not already started
    if (!m_autoTimerStarted)
    {
        m_timer.Reset();
        m_timer.Start();
        m_autoTimerStarted = true;
        printf("Starting auto mode\n");
    }

    if (!m_autoMode->IsAutoComplete())
    {
        m_robotStateMachine->AlwaysRunShooter();

        m_autoMode->RunActions();
    }
    else if (!m_autoModeCompleteMessagePrinted)
    {
        // * Print 'done' message only once
        printf("Auto mode complete\n");
        m_timer.Stop();
        double elapsedTimeSeconds = m_timer.Get().value();
        fmt::print("Auto mode took: {:.3f} seconds\n", elapsedTimeSeconds);
        m_autoModeCompleteMessagePrinted = true;
        m_autoTimerStarted = false;
    }

    if (!m_autoModeOvertimeWarningPrinted && m_timer.HasElapsed(units::second_t(15)))
    {
        printf("Auto mode took longer than 15 seconds.\n");
        m_autoModeOvertimeWarningPrinted = true;
    }
}

Autonomous *Autonomous::m_autonomous = nullptr;