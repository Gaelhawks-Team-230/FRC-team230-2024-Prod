// #pragma once

// #include "auto/modes/AutoMode.h"
// #include "auto/actions/ParallelAction.h"
// #include "auto/actions/SeriesAction.h"
// #include "auto/actions/WaitForTimeAction.h"
// #include "auto/actions/TrajectoryAction.h"
// #include "auto/actions/IntakeAction.h"
// #include "auto/actions/PrepShooterAction.h"
// #include "auto/actions/ShootAction.h"

// class ThreeNoteBetween : public AutoMode
// {
// public:
//     /**
//      * @brief Shoots preloaded & notes 8 & 9, staying out of the way of closest 3 notes
//      *
//      */
//     ThreeNoteBetween()
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

//         // START POS = center

//         // NOTE 1
//         AddAction(new PrepShooterAction(0.5, 3.0));
//         AddAction(new ShootAction(1.0));

//         //NOTE 2
//         TrajectoryAction *path1 = new TrajectoryAction({{55.0, 224.0, 0.0}, {75.0, 240.0, 0.0}, {113.0, 240.0, 0.0}, {306.0, 286.0, 0.0}, {315.0, 295.0, 0.0}}, l_normConstraints);  
//         AddAction(new ParallelAction( {path1, new SeriesAction({new WaitForTimeAction(1.2), new IntakeAction(1.8)})} ));
//         TrajectoryAction *path2 = new TrajectoryAction({{315.0, 295.0, 0.0}, {130.0, 273.0, -15.0}, {108.0, 273.0, -25.0}}, l_normConstraints);
//         AddAction(new ParallelAction({new PrepShooterAction(0.5, 7.2), path2}));
//         AddAction(new ShootAction(1.0));

//         // NOTE 3
//         TrajectoryAction *path3 = new TrajectoryAction({{108.0, 273.0, -25.0}, {306.0, 233.0, 15.0}}, l_normConstraints);
//         AddAction(new ParallelAction( {path3, new SeriesAction({new WaitForTimeAction(1.2), new IntakeAction(1.8)})} ));
//         TrajectoryAction *path4 = new TrajectoryAction({{306.0, 233.0, 15.0}, {108.0, 273.0, -25.0}}, l_normConstraints);
//         AddAction(new ParallelAction({new PrepShooterAction(0.5, 7.2), path4}));
//         AddAction(new ShootAction(1.0));


//         // AddAction(new TrajectoryAction({{55.0, 224.0, 0.0}, {75.0, 240.0, 0.0}, {113.0, 240.0, 0.0}, {306.0, 286.0, 0.0}}, l_normConstraints)); //? move between pieces, move to 4
//         // todo pickup 4
//         // AddAction(new TrajectoryAction({{306.0, 286.0, 0.0}, {194.0, 265.0, -30.0}}, l_normConstraints)); //? move up in wing
//         // todo shoot 4
//         // AddAction(new TrajectoryAction({{194.0, 265.0, -30.0}, {306.0, 233.0, 15.0}}, l_normConstraints)); //? move to 5
//         // todo pickup 5
//         // AddAction(new TrajectoryAction({{306.0, 233.0, 15.0}, {194.0, 265.0, -30.0}}, l_normConstraints)); //? move up in wing
//     }
// };