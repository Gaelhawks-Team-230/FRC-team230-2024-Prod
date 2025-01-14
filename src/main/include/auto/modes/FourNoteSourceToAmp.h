#pragma once

#include "auto/modes/AutoMode.h"
#include "auto/actions/ParallelAction.h"
#include "auto/actions/SeriesAction.h"
#include "auto/actions/WaitForTimeAction.h"
#include "auto/actions/TrajectoryAction.h"
#include "auto/actions/IntakeAction.h"
#include "auto/actions/PrepShooterAction.h"
#include "auto/actions/ShootAction.h"

class FourNoteSourceToAmp : public AutoMode
{
public:
    /**
     * @brief Shoots preloaded & closest 3 notes.
     *        In order from source side to amp side.
     */
    FourNoteSourceToAmp()
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
        TrajectoryAction *path4 = new TrajectoryAction({{74.0, 200.0, 10.0}, {74.0, 224.0, 0.0}, {99.0, 224.0, 0.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.6), path4}), new IntakeAction(2.1)}));
        TrajectoryAction *path5 = new TrajectoryAction({{99.0, 224.0, 0.0}, {74.0, 260.0, -25.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new IntakeAction(0.7), new PrepShooterAction(0.5, 55.5)}), path5}));
        AddAction(new ShootAction(1.0));
        // NOTE 4
        TrajectoryAction *path6 = new TrajectoryAction({{74.0, 260.0, -25.0}, {108.0, 273.0, -25.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.6), path6}), new IntakeAction(2.4)}));
        AddAction(new PrepShooterAction(0.8, 50.65));
        AddAction(new ShootAction(1.0));





        // AddAction(new TrajectoryAction({{55.0, 161.0, 0.0}, {86.0, 175.0, 25.0}}, l_normConstraints)); //? rotate and move to 3
        // TODO intake 3
        // TODO shoot 3
        // AddAction(new TrajectoryAction({{90.0, 166.0, 30.0}, {75.0, 181.0, 15.0}, {75.0, 196.0, 0.0}, {90.0, 211.0, 0.0}}, l_normConstraints)); //? traj move to 2
        // TODO intake 2
        // TODO shoot 2
        // AddAction(new TrajectoryAction({{90.0, 211.0, 0.0}, {75.0, 227.0, -15.0}, {75.0, 244.0, -30.0}, {90.0, 260.0, -30.0}}, l_normConstraints)); //? traj move to 1
        // TODO intake 1
        // TODO shoot 1

        //! TEST
        // AddAction(new TrajectoryAction({{88.0, -161.0, 0.0}, {82.0, -186.0, 0.0}, {88.0, -217.0, 0.0}}, l_slowConstraints));
        // AddAction(new TrajectoryAction({{88.0, -217.0, 0.0}, {82.0, -247.0, 0.0}, {88.0, -276.0, 0.0}}, l_slowConstraints));
    }
};