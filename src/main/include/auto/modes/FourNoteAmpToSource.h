// #pragma once

// #include "auto/modes/AutoMode.h"
// #include "auto/actions/TrajectoryAction.h"

// class FourNoteAmpToSource : public AutoMode
// {
// public:
//     /**
//      * @brief Shoots preloaded & closest 3 notes.
//      *       In order from source side to amp side.
//      */
//     FourNoteAmpToSource()
//     {
//         Tconstraints l_normConstraints;
//         l_normConstraints.amax = 100.0;
//         l_normConstraints.vmax = 75.0;
//         l_normConstraints.vmin = 0.0;

//         Tconstraints l_slowConstraints;
//         l_slowConstraints.amax = 20.0;
//         l_slowConstraints.vmax = 75.0;
//         l_slowConstraints.vmin = 0.0;

//         SetInitialRobotHeading(-60.0);

//         //* START POS = amp side against subwoofer
//         // TODO shoot preloaded
//         AddAction(new TrajectoryAction({{28.0, 263.0, -60.0}, {59.0, 262.0, -30.0}, {90.0, 260.0, -30.0}}, l_normConstraints)); //? rotate + pickup 1
//         // TODO shoot 1
//         AddAction(new TrajectoryAction({{90.0, 260.0, -30.0}, {75.0, 246.0, -15.0}, {75.0, 231.0, 0.0}, {90.0, 217.0, 0.0}}, l_normConstraints)); //? traj move to 2
//         // TODO shoot 2
//         AddAction(new TrajectoryAction({{90.0, 217.0, 0.0}, {75.0, 202.0, 15.0}, {75.0, 187.0, 30.0}, {90.0, 172.0, 30.0}}, l_normConstraints)); //? traj move to 3
//         // TODO shoot 3
//     }
// };