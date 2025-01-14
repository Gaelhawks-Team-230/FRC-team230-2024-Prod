#pragma once

#include "auto/modes/AutoMode.h"
#include "auto/actions/ParallelAction.h"
#include "auto/actions/SeriesAction.h"
#include "auto/actions/WaitForTimeAction.h"
#include "auto/actions/TrajectoryAction.h"
#include "auto/actions/IntakeAction.h"
#include "auto/actions/PrepShooterAction.h"
#include "auto/actions/ShootAction.h"
#include "auto/actions/SetShootFloor.h"
#include "auto/actions/SetOutOfFrameShooting.h"

class FiveNoteAmpSide : public AutoMode
{
public:
    /**
     * @brief Auto mode for the five note path on the amp of the field.
     *
     */
    FiveNoteAmpSide()
    {
        Tconstraints l_normConstraints;
        l_normConstraints.amax = 125.0;
        l_normConstraints.vmax = 100.0;
        l_normConstraints.vmin = 0.0;

        Tconstraints l_zoomConstraints; // to make paths faster. modify constraints as needed.
        l_zoomConstraints.amax = 170.0;
        l_zoomConstraints.vmax = 200.0;
        l_zoomConstraints.vmin = 0.0;

        SetInitialRobotHeading(0.0);

        //* START POS = Amp Side
        AddAction(new SetShootFloor());

        // NOTE 1
        TrajectoryAction *path1 = new TrajectoryAction({{55.0, 281.0, 0.0}, {81.0, 265.0, -25.0}}, l_normConstraints);
        AddAction(new WaitForTimeAction(0.5));
        AddAction(path1);
        AddAction(new ShootAction(1.0));

        // NOTE 2
        TrajectoryAction *path2 = new TrajectoryAction({{81.0, 265.0, -25.0}, {87.0, 271.0, -25.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.2), path2}), new IntakeAction(0.9)}));
        TrajectoryAction *path3 = new TrajectoryAction({{87.0, 271.0, -25.0}, {81.0, 265.0, -25.0}}, l_normConstraints);
        AddAction(new ParallelAction({new IntakeAction(0.3), path3, new SetOutOfFrameShooting(0.3)}));
        AddAction(new SetShootFloor());
        AddAction(new ShootAction(1.0));

        // NOTE 3
        TrajectoryAction *path4 = new TrajectoryAction({{81.0, 265.0, -25.0}, {194.0, 284.0, -15.0}, {280.0, 292.0, -5.0}, {331.0, 305.0, 0.0}}, l_zoomConstraints);
        AddAction(new ParallelAction({path4, new SetOutOfFrameShooting(2.6), new SeriesAction({new WaitForTimeAction(0.8), new IntakeAction(1.8)})}));
        TrajectoryAction *path5 = new TrajectoryAction({{331.0, 305.0, 0.0}, {130.0, 273.0, -10.0}, {81.0, 265.0, -25.0}}, l_zoomConstraints);
        AddAction(new ParallelAction({new SeriesAction({new IntakeAction(0.5), new SetShootFloor()}), path5, new SetOutOfFrameShooting(0.5)}));
        AddAction(new ShootAction(1.0));

        // NOTE 4
        TrajectoryAction *path6 = new TrajectoryAction({{81.0, 265.0, -25.0}, {75.0, 245.0, -10.0}, {72.0, 230.0, 0.0}, {105.0, 230.0, 0.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new IntakeAction(1.6), new SetShootFloor()}), path6, new SetOutOfFrameShooting(1.6)}));
        TrajectoryAction *path7 = new TrajectoryAction({{105.0, 230.0, 0.0}, {98.0, 230.0, 0.0}}, l_normConstraints);
        AddAction(path7);
        // AddAction(new WaitForTimeAction(0.5));
        AddAction(new ShootAction(1.0));

        // NOTE 5
        // TrajectoryAction *path6 = new TrajectoryAction({{108.0, 216.0, 0.0}, {167.0, 243.0, 0.0}, {230.0, 259.0, 0.0}, {274.0, 243.0, 0.0}, {320.0, 225.0, 0.0}}, l_normConstraints);
        // AddAction(new ParallelAction( {path6, new SeriesAction({new WaitForTimeAction(0.8), new IntakeAction(1.8)})} ));
        // TrajectoryAction *path7 = new TrajectoryAction({{320.0, 225.0, 0.0}, {274.0, 243.0, 0.0}, {230.0, 259.0, 0.0}, {167.0, 243.0, 0.0}, {108.0, 216.0, 0.0}}, l_normConstraints);
        // AddAction(new ParallelAction( {new SeriesAction({new IntakeAction(0.7), new PrepShooterAction(0.5, 7.2)}), path7} ));
        // AddAction(new ShootAction(1.0));
    }
};

// #pragma once

// #include "auto/modes/AutoMode.h"
// #include "auto/actions/TrajectoryAction.h"
// #include "auto/actions/IntakeAction.h"
// #include "auto/actions/PrepShooterAction.h"
// #include "auto/actions/ShootAction.h"

// class FiveNoteAmp : public AutoMode
// {
// public:
//     /**
//      * @brief Auto mode for the five note path on the right side of the field.
//      *
//      */
//     FiveNoteAmp()
//     {
//         Tconstraints l_normConstraints;
//         l_normConstraints.amax = 200.0;
//         l_normConstraints.vmax = 200.0;
//         l_normConstraints.vmin = 0.0;

//         Tconstraints l_slowConstraints;
//         l_slowConstraints.amax = 20.0;
//         l_slowConstraints.vmax = 75.0;
//         l_slowConstraints.vmin = 0.0;

//         SetInitialRobotHeading(0.0);

//         //* START POS = center
//         // todo shoot preloaded
//         AddAction(new TrajectoryAction({{53.0, 219.0, 0.0}, {90.0, 219.0, 0.0}}, l_normConstraints)); //?  move to 2
//         // todo pickup 2, shoot 2
//         AddAction(new TrajectoryAction({{90.0, 219.0, 0.0}, {75.0, 227.0, -15.0}, {75.0, 244.0, -30.0}, {90.0, 260.0, -30.0}}, l_normConstraints)); //? move to 3
//         // todo pickup 3, shoot 3
//         AddAction(new TrajectoryAction({{90.0, 260.0, -30.0}, {290.0, 288.0, 0.0}, {303.0, 288.0, 0.0}}, l_normConstraints)); //? move to 9
//         // todo pickup 9
//         AddAction(new TrajectoryAction({{303.0, 288.0, 0.0}, {197.0, 260.0, 0.0}}, l_normConstraints)); //? move up in wing
//         // todo shoot 9
//         AddAction(new TrajectoryAction({{197.0, 260.0, 0.0}, {303.0, 212.0, 0.0}}, l_normConstraints)); //? move to 8
//         // todo pickup 8
//         AddAction(new TrajectoryAction({{303.0, 212.0, 0.0}, {197.0, 260.0, 0.0}}, l_normConstraints)); //? move up in wing
//         // todo shoot 8
//     }
// };