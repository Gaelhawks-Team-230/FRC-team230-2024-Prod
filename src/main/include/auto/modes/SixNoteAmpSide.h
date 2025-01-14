// #pragma once

// #include "auto/modes/AutoMode.h"
// #include "auto/actions/TrajectoryAction.h"

// class SixNoteAmpSide : public AutoMode
// {
// public:
//     /**
//      * @brief PLEASE WORK :sob:
//      *
//      */
//     SixNoteAmpSide()
//     {
//         Tconstraints l_normConstraints;
//         l_normConstraints.amax = 100.0;
//         l_normConstraints.vmax = 75.0;
//         l_normConstraints.vmin = 0.0;

//         Tconstraints l_slowConstraints;
//         l_slowConstraints.amax = 20.0;
//         l_slowConstraints.vmax = 75.0;
//         l_slowConstraints.vmin = 0.0;

//         Tconstraints l_zoomConstraints;
//         l_zoomConstraints.amax = 100.0;
//         l_zoomConstraints.vmax = 200.0;
//         l_zoomConstraints.vmin = 0.0;

//         SetInitialRobotHeading(0.0);

//         //* START POS = source side against subwoofer
//         // todo shoot preloaded 
//         AddAction(new TrajectoryAction({{53.0, 219.0, 0.0}, {90.0, 219.0, 0.0}}, l_normConstraints)); //?  move to 2
//         // todo pickup 2, shoot 2
//         AddAction(new TrajectoryAction({{90.0, 219.0, 0.0}, {80.0, 247.0, 0.0}, {90.0, 276.0, 0.0}}, l_normConstraints)); //? move to 3
//         // todo pickup 3, shoot 3
//         AddAction(new TrajectoryAction({{90.0, 276.0, 0.0}, {303.0, 303.0, 0.0}}, l_normConstraints)); //? move to 9
//         // todo pickup 9
//         AddAction(new TrajectoryAction({{303.0, 293.0, 0.0}, {197.0, 260.0, 0.0}}, l_normConstraints)); //? move up in wing
//         // todo shoot 9
//         AddAction(new TrajectoryAction({{197.0, 260.0, 0.0}, {303.0, 237.0, 0.0}}, l_normConstraints)); //? move to 8 
//         // todo pickup 8
//         AddAction(new TrajectoryAction({{303.0, 237.0, 0.0}, {197.0, 260.0, 0.0}}, l_normConstraints)); //? move up in wing
//         // todo shoot 8
//         AddAction(new TrajectoryAction({{197.0, 260.0, 0.0}}, l_normConstraints)); //? move to 7
//         // todo pickup 7
//         AddAction(new TrajectoryAction({}, l_normConstraints)); //? move up in wing
//         // todo shoot
//     }
// };