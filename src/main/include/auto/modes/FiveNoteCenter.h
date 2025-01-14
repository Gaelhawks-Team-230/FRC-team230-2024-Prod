// #pragma once

// #include "auto/modes/AutoMode.h"
// #include "auto/actions/TrajectoryAction.h"
// #include "auto/actions/IntakeAction.h"
// #include "auto/actions/PrepShooterAction.h"
// #include "auto/actions/ShootAction.h"

// class FiveNoteCenter : public AutoMode
// {
// public:
//     /**
//      * @brief Auto mode for the five note path on the right side of the field.
//      *
//      */
//     FiveNoteCenter()
//     {
//         Tconstraints l_normConstraints;
//         l_normConstraints.amax = 100.0;
//         l_normConstraints.vmax = 75.0;
//         l_normConstraints.vmin = 0.0;

//         Tconstraints l_slowConstraints;
//         l_slowConstraints.amax = 20.0;
//         l_slowConstraints.vmax = 75.0;
//         l_slowConstraints.vmin = 0.0;

//         SetInitialRobotHeading(0.0);

//         //* START POS = center
//         // todo shoot preloaded
//     }
// };