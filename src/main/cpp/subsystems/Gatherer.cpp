#include "subsystems/Gatherer.h"

/**
 * @brief Gets instance of gatherer
 *
 * @return Gatherer*
 */
Gatherer *Gatherer::GetInstance()
{
    if (m_gatherer == nullptr)
    {
        m_gatherer = new Gatherer();
    }
    return m_gatherer;
}

/**
 * @brief Construct a new Gatherer:: Gatherer object
 *
 */
Gatherer::Gatherer()
{
    m_intakeMotor.RestoreFactoryDefaults();
    m_intakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_EntryBeamBreak = new frc::DigitalInput(Constants::DigitalIO::ENTRY_BEAM_BREAK);
    m_StorageBeamBreak = new frc::DigitalInput(Constants::DigitalIO::STORAGE_BEAM_BREAK);
    LocalReset();
}

/**
 * @brief Resets all relevant values
 *
 */
void Gatherer::LocalReset()
{
    m_isEntryBeamBroken = false;
    m_isStorageBeamBroken = false;
    m_intakeCmd = 0.0;
    m_storagePersistanceCounter = 0;
}

/**
 * @brief Stops motor function and resets values
 *
 */
void Gatherer::Stop()
{
    m_intakeMotor.StopMotor();
    LocalReset();
}

/**
 * @brief Reads sensor values
 *
 */
void Gatherer::Analyze()
{
    m_isEntryBeamBroken = !(m_EntryBeamBreak->Get());
    bool isStorageBeamBroken = !(m_StorageBeamBreak->Get());

    if (isStorageBeamBroken)
    {
        m_storagePersistanceCounter++;
    }
    else
    {
        m_isStorageBeamBroken = false;
        m_storagePersistanceCounter = 0;
    }

    if (m_storagePersistanceCounter > STORAGE_BEAM_BREAK_DELAY)
    {
        m_isStorageBeamBroken = true;
    }
}

/**
 * @brief Sets intake motor speed
 *
 */
void Gatherer::Periodic()
{
    m_intakeMotor.Set(m_intakeCmd);
}

/**
 * @brief Initial stage of piece gathering with faster motor speed
 *
 */
void Gatherer::GatherPieceFast()
{
    m_intakeCmd = FAST_COLLECT_CMD;
}

/**
 * @brief Second stage of piece gathering with slower motor speed after piece is in the intake
 *
 */
void Gatherer::GatherPieceSlow()
{
    m_intakeCmd = SLOW_COLLECT_CMD;
}

/**
 * @brief Ejects piece for whatever reason at a higher speed
 *
 */
void Gatherer::EjectPiece()
{
    m_intakeCmd = EJECT_CMD;
}

/**
 * @brief Feeds game piece into the intake
 *
 */
void Gatherer::ReleasePiece()
{
    m_intakeCmd = RELEASE_CMD;
}

/**
 * @brief Stops the intake motor
 *
 */
void Gatherer::StopIntake()
{
    m_intakeCmd = 0.0;
}

/**
 * @brief sends values to the smartdashboard
 *
 */
void Gatherer::UpdateDash()
{
    frc::SmartDashboard::PutBoolean("Gatherer/IsEntryBeamBroken", m_isEntryBeamBroken);
    frc::SmartDashboard::PutBoolean("Gatherer/IsStorageBeamBroken", m_isStorageBeamBroken);
    frc::SmartDashboard::PutNumber("Gatherer/MotorCmd", m_intakeCmd);
}

Gatherer *Gatherer::m_gatherer = nullptr;