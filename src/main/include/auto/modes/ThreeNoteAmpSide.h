#pragma once

#include "auto/modes/AutoMode.h"
#include "auto/actions/ParallelAction.h"
#include "auto/actions/SeriesAction.h"
#include "auto/actions/WaitForTimeAction.h"
#include "auto/actions/TrajectoryAction.h"
#include "auto/actions/IntakeAction.h"
#include "auto/actions/PrepShooterAction.h"
#include "auto/actions/ShootAction.h"

class ThreeNoteAmpSide : public AutoMode
{
public:
    /**
     * @brief Shoots preloaded & closest 3 notes.
     *
     */
    ThreeNoteAmpSide()
    {
        Tconstraints l_normConstraints;
        l_normConstraints.amax = 100.0;
        l_normConstraints.vmax = 120.0;
        l_normConstraints.vmin = 0.0;

        Tconstraints l_zoomConstraints;
        l_zoomConstraints.amax = 150.0;
        l_zoomConstraints.vmax = 180.0;
        l_zoomConstraints.vmin = 0.0;

        Tconstraints l_slowConstraints;
        l_slowConstraints.amax = 20.0;
        l_slowConstraints.vmax = 75.0;
        l_slowConstraints.vmin = 0.0;

        SetInitialRobotHeading(0.0);

        // NOTE 1
        TrajectoryAction *path1 = new TrajectoryAction({{55.0, 281.0, 0.0}, {55.0, 263.0, -25.0}}, l_normConstraints);  
        AddAction(new ParallelAction({path1, new PrepShooterAction(0.5, 56.5)}));
        AddAction(new ShootAction(1.0));
        // NOTE 2
        TrajectoryAction *path2 = new TrajectoryAction({{55.0, 263.0, -25.0}, {108.0, 279.0, -25.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.4), path2}), new IntakeAction(1.8)}));
        AddAction(new PrepShooterAction(0.8, 50.0));
        AddAction(new ShootAction(1.0));
        // NOTE 3
        // fix final traj - 4 inches right 
        TrajectoryAction *path3 = new TrajectoryAction({{108.0, 279.0, -25.0}, {194.0, 284.0, -15.0}, {280.0, 292.0, -5.0}, {315.0, 299.0, 0.0}}, l_normConstraints);
        AddAction(new ParallelAction( {path3, new SeriesAction({new WaitForTimeAction(0.8), new IntakeAction(1.8)})} ));
        TrajectoryAction *path4 = new TrajectoryAction({{315.0, 299.0, 0.0}, {130.0, 273.0, -15.0}, {108.0, 273.0, -25.0}}, l_normConstraints);
        AddAction(new ParallelAction( {new SeriesAction({new IntakeAction(0.7), new PrepShooterAction(0.5, 50.0)}), path4} ));
        AddAction(new ShootAction(1.0));
    }
};