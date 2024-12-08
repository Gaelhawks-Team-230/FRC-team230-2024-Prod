#pragma once

#include "frc/DriverStation.h"
#include <frc/geometry/Pose3d.h>

namespace Constants
{

    const double LOOPTIME = 0.02;

    const frc::DriverStation::Alliance DEFAULT_ALLIANCE = frc::DriverStation::Alliance::kRed;

    const bool CTRE_FACTORY_RESET = false;
    const bool SWERVE_CALIBRATION_MODE = false;
    const bool SWERVE_SYS_ID = false;

    namespace Swerve
    {
        const double R_SFACTOR = 200.0;
        const double XY_SFACTOR = 200.0;
        const double R_SFACTOR_LOW = 100.0;
        const double XY_SFACTOR_LOW = 100.0;
    }

    namespace CAN
    {
        const int PDP_MODULE = 0;

        // * Swerve
        // Module 1
        const int FRONT_RIGHT_DRIVE = 1;
        const int FRONT_RIGHT_STEER = 5;
        const int FRONT_RIGHT_ABSOLUTE_ENCODER = 9;
        // Module 2
        const int BACK_RIGHT_DRIVE = 2;
        const int BACK_RIGHT_STEER = 6;
        const int BACK_RIGHT_ABSOLUTE_ENCODER = 10;
        // Module 3
        const int BACK_LEFT_DRIVE = 3;
        const int BACK_LEFT_STEER = 7;
        const int BACK_LEFT_ABSOLUTE_ENCODER = 11;
        // Module 4
        const int FRONT_LEFT_DRIVE = 4;
        const int FRONT_LEFT_STEER = 8;
        const int FRONT_LEFT_ABSOLUTE_ENCODER = 12;
    }

    namespace FlightCtrlAxis
    {
        const int X_AXIS = 1;
        const int Y_AXIS = 5;
        const int Z_AXIS = 0;

        const double X_DEADBAND = 0.1;
        const double Y_DEADBAND = 0.1;
        const double R_DEADBAND = 0.1;
        const double X_SHAPING = 0.6;
        const double Y_SHAPING = 0.6;
        const double R_SHAPING = 1.0;
    }

    namespace FlightCtrlButtons
    {
        const unsigned int GYRO_BUTTON_SWITCH = 1;
        const unsigned int FIELD_ROBOT_SWITCH = 2;
        const unsigned int RESET_GYRO_BUTTON = 3;
        // const unsigned int VISION_CENTER_BUTTON = 15;
        // const unsigned int VISION_LEFT_BUTTON = 14;
        // const unsigned int VISION_LEFT_BUTTON = 6;
        // const unsigned int VISION_RIGHT_BUTTON = 7;
        // const unsigned int VISION_DEBUG_BUTTON = 14;
    }

    namespace GamepadButtons
    {
        const unsigned int SHOOTING_POS_BUTTON = 1;
        const unsigned int STOW_POS_BUTTON = 2;
        const unsigned int AMP_POS_BUTTON = 3;
        const unsigned int TRAP_POS_BUTTON = 4;
        const unsigned int DROP_NOTE_BUTTON = 5;
        const unsigned int PICKUP_NOTE_BUTTON = 6;
        const unsigned int PREP_TO_SHOOT_BUTTON = 7;
        const unsigned int SHOOT_BUTTON = 8;
        const unsigned int AMPLIFY_SIGNAL_BUTTON = 9;
        const unsigned int NOTE_SIGNAL_BUTTON = 10;

    }

    namespace GamepadAxis
    {
        const int LEFT_CLIMBER_AXIS = 1;
        const int RIGHT_CLIMBER_AXIS = 3;
    }
    namespace POI
    {
        const frc::Pose3d RED_SPEAKER = frc::Pose3d(218.625_in, 651.25_in, 78.13_in, frc::Rotation3d(0_deg, 0_deg, 180_deg));
        const frc::Pose3d BLUE_SPEAKER = frc::Pose3d(218.625_in, 0_in, 78.13_in, frc::Rotation3d(0_deg, 0_deg, 0_deg));

        const double TRAP_APRILTAG_HEIGHT = 48.81;
        const double AMP_APRILTAG_HEIGHT = 50.13;
        const double SPEAKER_APRILTAG_HEIGHT = 53.88;
        // id 1&2 blue alliance source
        // id 3&4 red alliance speaker
        // id 5 red alliance amp
        // id 6 blue allaince amp
        // id 7&8 blue alliance speaker
        // id 9&10 red alliance source
        // id 11, 12, 13 are red alliance trap
        // id 14, 15, 16 are blue alliance trap

    }

}
