// #pragma once

// #include "auto/modes/AutoMode.h"
// #include "auto/actions/TrajectoryAction.h"

// class FourNoteCenter : public AutoMode
// {
// public:
//     /**
//      * @brief Shoots preloaded & closest 3 notes.
//      *
//      */
//     FourNoteCenter()
//     {
//         Tconstraints l_normConstraints;
//         l_normConstraints.amax = 100.0;
//         l_normConstraints.vmax = 120.0;
//         l_normConstraints.vmin = 0.0;

//         Tconstraints l_slowConstraints;
//         l_slowConstraints.amax = 20.0;
//         l_slowConstraints.vmax = 75.0;
//         l_slowConstraints.vmin = 0.0;

//         SetInitialRobotHeading(0.0);

//         //* START POS = center against subwoofer
//         // TODO shoot preloaded
//         AddAction(new TrajectoryAction({{53.0, 219.0, 0.0}, {98.0, 219.0, 0.0}}, l_normConstraints)); //? move to middle close note
//         // TODO pickup middle close note
//         // TODO shoot note
//         AddAction(new TrajectoryAction({{98.0, 217.0, 0.0}, {233.0, 264.0, 0.0}, {303.0, 222.0, 0.0}}, l_normConstraints)); //? move to far note 2
//         // TODO pickup note
//         AddAction(new TrajectoryAction({{303.0, 222.0, 0.0}, {202.0, 249.0, 0.0}, {170.0, 217.0, 0.0}}, l_normConstraints)); //? move up to shoot
//         // TODO shoot note
//         AddAction(new TrajectoryAction({{170.0, 217.0, 0.0}, {196.0, 168.0, 0.0}, {303.0, 152.0, 0.0}}, l_normConstraints)); //? circle around
//         // TODO pickup far note 3
//         AddAction(new TrajectoryAction({{303.0, 162.0, 0.0}, {150.0, 196.0, 0.0}}, l_normConstraints)); //? move up to shoot
//         // TODO shoot note
//     }
// };