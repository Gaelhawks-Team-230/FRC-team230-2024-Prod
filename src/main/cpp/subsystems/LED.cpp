#include "subsystems/LED.h"

/**
 * @brief Get instance of LED
 * 
 * @return LED* 
 */
LED *LED::GetInstance()
{
    if (m_led == nullptr)
    {
        m_led = new LED();
    }
    return m_led;
}

/**
 * @brief Construct a new LED::LED object
 * 
 */
LED::LED()
{
    // Sets the value of each output pin to the value inputted by the RoboRio
    ledOut0 = new frc::DigitalOutput(Constants::DigitalIO::LED_LIGHT_OUTPUT_1);
    ledOut1 = new frc::DigitalOutput(Constants::DigitalIO::LED_LIGHT_OUTPUT_2);
    ledOut2 = new frc::DigitalOutput(Constants::DigitalIO::LED_LIGHT_OUTPUT_3);

}

/**
 * @brief Reset the LED to a blank state
 * 
 */
void LED::LocalReset()
{
    // Display Blank
    DisplayBlank();
}

/**
 * @brief Display blank state - off/black | 000
 * 
 */
void LED::DisplayBlank()
{
    light0Status = false;
    light1Status = false;
    light2Status = false;
}

/**
 * @brief Display disconnected from DS state - blinking red | 001
 * 
 */
void LED::DisplayDisconnected()
{
    light0Status = false;
    light1Status = false;
    light2Status = true;
}
/**
 * @brief Display connected to DS state - green cyclon animation | 010
 * 
 */
void LED::DisplayConnected()
{
    light0Status = false;
    light1Status = true;
    light2Status = false;
}

/**
 * @brief Call out to human player to display the amplify state - flashing blue | 011
 * 
 */
void LED::DisplayAmplify()
{
    light0Status = false;
    light1Status = true;
    light2Status = true;
}

/**
 * @brief Call out to the human player to feed us a Note - flashing orange | 100
 * 
 */
void LED::DisplayGamepieceWanted()
{
    light0Status = true;
    light1Status = false;
    light2Status = false;
}

/**
 * @brief Brief flashing that signals when we aquire a gamepiece | 101
 * 
 */
void LED::DisplayHasGamepiece()
{
    light0Status = true;
    light1Status = false;
    light2Status = true;
}

/**
 * @brief Animation when we are hunting for a note | 110
 * 
 */
void LED::DisplayHunting()
{
    light0Status = true;
    light1Status = true;
    light2Status = false;
}

// Control the output of the led strips
// 2^3 = 8 different options
void LED::Periodic()
{
    ledOut0->Set(light0Status);
    ledOut1->Set(light1Status);
    ledOut2->Set(light2Status);
}

void LED::UpdateDash()
{
    frc::SmartDashboard::PutBoolean("LED/Light0", light0Status);
    frc::SmartDashboard::PutBoolean("LED/Light1", light1Status);
    frc::SmartDashboard::PutBoolean("LED/Light2", light2Status);
}
LED *LED::m_led = nullptr;