#pragma once

#include "frc/DriverStation.h"
#include <frc/geometry/Pose3d.h>

namespace Constants
{

    const double LOOPTIME = 0.02;

    const double DMIN = (1.0 / 1025.0);
    const double DMAX = (1024.0 / 1025.0);

    const frc::DriverStation::Alliance DEFAULT_ALLIANCE = frc::DriverStation::Alliance::kRed;

    const bool CTRE_FACTORY_RESET = false;
    const bool SWERVE_CALIBRATION_MODE = false;
    const bool SWERVE_SYS_ID = false;

    const bool SHOOT_CALIBRATION_MODE = false;

    namespace Swerve
    {
        const double R_SFACTOR = 250.0;
        const double XY_SFACTOR = 180.0;
        // const double XY_SFACTOR = 220.0;
        const double R_SFACTOR_LOW = 250.0;
        const double XY_SFACTOR_LOW = 72.0;
        // const double XY_SFACTOR_LOW = 220.0;
    }

    namespace CAN
    {
        const int PDP_MODULE = 1;

        // * Swerve
        // Module 1
        const int FRONT_RIGHT_DRIVE = 2;
        const int FRONT_RIGHT_STEER = 6;
        const int FRONT_RIGHT_ABSOLUTE_ENCODER = 10;
        // Module 2
        const int BACK_RIGHT_DRIVE = 3;
        const int BACK_RIGHT_STEER = 7;
        const int BACK_RIGHT_ABSOLUTE_ENCODER = 11;
        // Module 3
        const int BACK_LEFT_DRIVE = 4;
        const int BACK_LEFT_STEER = 8;
        const int BACK_LEFT_ABSOLUTE_ENCODER = 12;
        // Module 4
        const int FRONT_LEFT_DRIVE = 5;
        const int FRONT_LEFT_STEER = 9;
        const int FRONT_LEFT_ABSOLUTE_ENCODER = 13;

        const int LEFT_SHOOTER_FALCON = 14;
        const int RIGHT_SHOOTER_FALCON = 15;

        const int ARM_FALCON = 16;
        const int PLATFORM_FALCON = 17;
        const int INTAKE_SPARK = 18;
        const int RIGHT_CLIMBER_FALCON = 19;
        const int LEFT_CLIMBER_FALCON = 20;

    }

    namespace DigitalIO
    {
        const unsigned int ARM_ABSOLUTE_ENCODER = 0;
        const unsigned int PLATFORM_ABSOLUTE_ENCODER = 1;
        const unsigned int ENTRY_BEAM_BREAK = 2;
        const unsigned int STORAGE_BEAM_BREAK = 3;
        const unsigned int LED_LIGHT_OUTPUT_1 = 4;
        const unsigned int LED_LIGHT_OUTPUT_2 = 5;
        const unsigned int LED_LIGHT_OUTPUT_3 = 6;
    }

    namespace FlightCtrlAxis
    {
        const int X_AXIS = 1;
        const int Y_AXIS = 5;
        const int Z_AXIS = 0;

        const double X_DEADBAND = 0.1;
        const double Y_DEADBAND = 0.1;
        const double R_DEADBAND = 0.1;

        // const double X_SHAPING = 0.6;
        // const double Y_SHAPING = 0.6;
        // const double R_SHAPING = 1.0;

        const double X_SHAPING = 1.0;
        const double Y_SHAPING = 1.0;
        const double R_SHAPING = 1.0;
    }

    namespace FlightCtrlButtons
    {
        const unsigned int GYRO_BUTTON_SWITCH = 1;
        const unsigned int FIELD_ROBOT_SWITCH = 2;
        const unsigned int RESET_GYRO_BUTTON = 3;
        // const unsigned int SHOOTER_TEST = 14;
        const unsigned int AMP_ALIGN_BUTTON_1 = 6;
        const unsigned int AMP_ALIGN_BUTTON_2 = 7;
        const unsigned int SHOOTER_CAL_BUTTON = 15;
        // const unsigned int VISION_LEFT_BUTTON = 6;
        // const unsigned int VISION_RIGHT_BUTTON = 7;
        // const unsigned int VISION_DEBUG_BUTTON = 14;
    }

    namespace GamepadButtons
    {
        const unsigned int HIGH_TRAP_POS_BUTTON = 1; // TODO UPDATE
        const unsigned int STOW_POS_BUTTON = 2;      // TODO UPDATE
        const unsigned int AMP_POS_BUTTON = 3;
        const unsigned int TRAP_POS_BUTTON = 4;
        const unsigned int EJECT_NOTE_BUTTON = 5;
        const unsigned int PICKUP_NOTE_BUTTON = 6;
        const unsigned int PREP_TO_SHOOT_BUTTON = 7;
        const unsigned int SHOOT_BUTTON = 8;
        const unsigned int CLIMB_LOCK_BUTTON = 10;

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
