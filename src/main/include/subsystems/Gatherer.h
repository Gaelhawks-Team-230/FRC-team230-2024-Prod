#pragma once

#include "rev/CANSparkMax.h"
#include "rev/CanSparkBase.h"

#include "frc/DigitalInput.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "util/Subsystem.h"

const double FAST_COLLECT_CMD = 0.8;
const double SLOW_COLLECT_CMD = 0.4; // 0.6 old val
const double EJECT_CMD = -0.95;
const double RELEASE_CMD = 1.0;

const unsigned int STORAGE_BEAM_BREAK_DELAY = 0; // we changed this from 8 to 6 to 0

class Gatherer : public Subsystem
{
private:
    Gatherer();
    static Gatherer *m_gatherer;

    rev::CANSparkMax m_intakeMotor{Constants::CAN::INTAKE_SPARK, rev::CANSparkMax::MotorType::kBrushless};

    frc::DigitalInput *m_EntryBeamBreak;
    frc::DigitalInput *m_StorageBeamBreak;

    bool m_isEntryBeamBroken;
    bool m_isStorageBeamBroken;
    double m_intakeCmd;
    unsigned int m_storagePersistanceCounter;

public:
    static Gatherer *GetInstance();

    /**
     * @brief returns true if the entry beam break is broken and false otherwise
     * 
     * @return true 
     * @return false 
     */
    bool IsEntryBeamBroken() { return m_isEntryBeamBroken; };
    
    /**
     * @brief returns true if the storage beam break is broken and false otherwise
     * 
     * @return true 
     * @return false 
     */
    bool IsStorageBeamBroken() { return m_isStorageBeamBroken; };
    void GatherPieceFast();
    void GatherPieceSlow();
    void EjectPiece();
    void StopIntake();
    void ReleasePiece(); // releases the gamepiece into the shooter

    void LocalReset() override;
    void Stop() override;
    void UpdateDash() override;
    void Analyze() override;
    void Periodic() override;
};