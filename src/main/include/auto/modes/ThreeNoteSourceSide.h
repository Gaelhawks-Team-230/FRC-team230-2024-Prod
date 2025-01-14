#pragma once

#include "auto/modes/AutoMode.h"
#include "auto/actions/ParallelAction.h"
#include "auto/actions/SeriesAction.h"
#include "auto/actions/WaitForTimeAction.h"
#include "auto/actions/TrajectoryAction.h"
#include "auto/actions/IntakeAction.h"
#include "auto/actions/PrepShooterAction.h"
#include "auto/actions/ShootAction.h"

class ThreeNoteSourceSide : public AutoMode
{
public:
    /**
     * @brief Shoots preloaded & closest 3 notes.
     *        In order from source side to amp side.
     */
    ThreeNoteSourceSide()
    {
        Tconstraints l_normConstraints;
        l_normConstraints.amax = 125.0;
        l_normConstraints.vmax = 100.0;
        l_normConstraints.vmin = 0.0;

        Tconstraints l_slowConstraints;
        l_slowConstraints.amax = 20.0;
        l_slowConstraints.vmax = 75.0;
        l_slowConstraints.vmin = 0.0;

        SetInitialRobotHeading(0.0);

        //* START POS = source side against subwoofer
        // TODO shoot preloaded
        // NOTE 1
        TrajectoryAction *path1 = new TrajectoryAction({{55.0, 161.0, 0.0}, {74.0, 181.0, 25.0}}, l_normConstraints);
        AddAction(new ParallelAction({path1, new PrepShooterAction(0.5, 53.0)}));
        AddAction(new ShootAction(1.0));
        TrajectoryAction *path2 = new TrajectoryAction({{74.0, 181.0, 25.0}, {92.0, 169.0, 25.0}}, l_normConstraints);
        // NOTE 2
        AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.6), path2}), new IntakeAction(2.0)}));
        TrajectoryAction *path3 = new TrajectoryAction({{92.0, 169.0, 25.0}, {74.0, 181.0, 15.0}, {74.0, 200.0, 10.0}}, l_normConstraints);
        AddAction(new ParallelAction({path3, new PrepShooterAction(0.5, 55.0)}));
        AddAction(new ShootAction(1.0));
        // NOTE 3
        TrajectoryAction *path4 = new TrajectoryAction({{74.0, 200.0, 10.0}, {100.0, 126.0, 0.0}, {186.0, 74.0, 0.0}, {251.0, 81.0, 0.0}, {290.0, 112.0, 0.0}, {315.0, 109.0, 0.0}}, l_normConstraints);
        AddAction(new ParallelAction({path4, new IntakeAction(4.0)}));
        TrajectoryAction *path5 = new TrajectoryAction({{315.0, 109.0, 0.0}, {290.0, 112.0, 0.0}, {251.0, 81.0, 0.0}, {186.0, 74.0, 0.0}, {100.0, 126.0, 10.0}, {74.0, 212.0, 25.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new IntakeAction(0.7), new PrepShooterAction(3.5, 56.0)}), path5}));
        AddAction(new ShootAction(1.0));

    }
};






// #pragma once

// #include "auto/modes/AutoMode.h"
// #include "auto/actions/TrajectoryAction.h"

// class ThreeNoteSourceSide : public AutoMode
// {
// public:
//     /**
//      * @brief Shoots preloaded & notes 4 & 5.
//      *
//      */
//     ThreeNoteSourceSide()
//     {
//         Tconstraints l_normConstraints;
//         l_normConstraints.amax = 100.0;
//         l_normConstraints.vmax = 100.0;
//         l_normConstraints.vmin = 0.0;

//         Tconstraints l_slowConstraints;
//         l_slowConstraints.amax = 20.0;
//         l_slowConstraints.vmax = 75.0;
//         l_slowConstraints.vmin = 0.0;

//         SetInitialRobotHeading(60.0);

//         //* START POS = source side against subwoofer
//         // TODO shoot preloaded
//         AddAction(new TrajectoryAction({{28.0, 174.0, 60.0}, {134.0, 69.0, 30.0}, {298.0, 41.0, 10.0}}, l_normConstraints)); //? rotate + move to 9
//         // todo pickup 9 
//         AddAction(new TrajectoryAction({{298.0, 31.0, 10.0}, {141.0, 93.0, 40.0}}, l_normConstraints)); //? move up in wing 
//         // todo shoot 9
//         AddAction(new TrajectoryAction({{141.0, 93.0, 40.0}, {232.0, 63.0, 30.0}, {298.0, 93.0, 0.0}}, l_normConstraints)); //? move to 8
//         // todo pickup 8
//         AddAction(new TrajectoryAction({{298.0, 93.0, 0.0}, {232.0, 63.0, 30.0}, {198.0, 75.0, 40.0}}, l_normConstraints)); //? move up in wing
//         // todo shoot 8
//     }
// };