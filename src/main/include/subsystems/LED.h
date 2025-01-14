#pragma once

#include <frc/DigitalOutput.h>
#include "Constants.h"
#include "util/Subsystem.h"
#include "subsystems/LED.h"
#include <frc/smartdashboard/SmartDashboard.h>

class LED : public Subsystem
{
private:
    LED();
    static LED *m_led;
    // Digital Outputs
    frc::DigitalOutput *ledOut0;
    frc::DigitalOutput *ledOut1;
    frc::DigitalOutput *ledOut2;

    bool light0Status;
    bool light1Status;
    bool light2Status;

public:
    static LED *GetInstance();

    void LocalReset(void) override;
    void Stop(void) override {};
    void UpdateDash(void) override;
    void Analyze(void) override {};
    void Periodic(void) override;

    // LED
    void DisplayBlank(void);
    void DisplayDisconnected(void);
    void DisplayConnected(void);
    void DisplayAmplify(void);
    void DisplayGamepieceWanted(void);
    void DisplayHasGamepiece(void);
    void DisplayHunting(void);
};