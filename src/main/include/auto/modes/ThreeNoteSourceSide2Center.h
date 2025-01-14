#pragma once

#include "auto/modes/AutoMode.h"
#include "auto/actions/ParallelAction.h"
#include "auto/actions/SeriesAction.h"
#include "auto/actions/WaitForTimeAction.h"
#include "auto/actions/TrajectoryAction.h"
#include "auto/actions/IntakeAction.h"
#include "auto/actions/PrepShooterAction.h"
#include "auto/actions/ShootAction.h"

class ThreeNoteSourceSide2Center : public AutoMode
{
public:
    /**
     * @brief Shoots preloaded & closest 3 notes.
     *        In order from source side to amp side.
     */
    ThreeNoteSourceSide2Center()
    {
        Tconstraints l_normConstraints;
        l_normConstraints.amax = 125.0;
        l_normConstraints.vmax = 150.0;
        l_normConstraints.vmin = 0.0;

        Tconstraints l_zoomConstraints;
        l_zoomConstraints.amax = 150.0;
        l_zoomConstraints.vmax = 150.0;
        l_zoomConstraints.vmin = 0.0;

        Tconstraints l_slowConstraints;
        l_slowConstraints.amax = 75.0;
        l_slowConstraints.vmax = 50.0;
        l_slowConstraints.vmin = 0.0;

        SetInitialRobotHeading(0.0);

        //* START POS = between two DS walls and against auto line

        // NOTE 1
        //TODO figure out starting pos
        TrajectoryAction *path1 = new TrajectoryAction({{55.0, 116.0, 0.0}, {105.0, 126.0, 43.0}}, l_normConstraints); // start to shoot note 1
        AddAction(new ParallelAction({path1, new PrepShooterAction(0.5, 49.0)}));
        AddAction(new ShootAction(0.5));

        // NOTE 2  
        TrajectoryAction *path2 = new TrajectoryAction({{105.0, 126.0, 43.0}, {186.0, 74.0, 15.0}, {251.0, 81.0, 0.0}, {290.0, 101.0, 0.0}, {315.0, 101.0, 0.0}}, l_normConstraints); // goto note 2
        // TrajectoryAction *path2end = new TrajectoryAction({{290.0, 101.0, 0.0}, {315.0, 101.0, 0.0}}, l_normConstraints);
        // AddAction(new ParallelAction({new SeriesAction({path2, path2end}), new IntakeAction(3.4)}));
        AddAction(new ParallelAction({path2, new IntakeAction(2.4)}));
        TrajectoryAction *path3 = new TrajectoryAction({{315.0, 101.0, 0.0}, {290.0, 101.0, 0.0}, {251.0, 81.0, 0.0}, {186.0, 74.0, 15.0}, {105.0, 132.0, 42.0}}, l_normConstraints); // come back and score
        AddAction(new ParallelAction({path3, new SeriesAction({new IntakeAction(0.6), new PrepShooterAction(1.9, 48.5)})}));
        AddAction(new ShootAction(0.7));
        // NOTE 3
        TrajectoryAction *path4 = new TrajectoryAction({{105.0, 132.0, 42.0}, {192.0, 50.0, 15.0}, {250.0, 40.0, 0.0}, {290.0, 40.0, 0.0}, {315.0, 40.0, 0.0}}, l_normConstraints); // goto note 3
        // TrajectoryAction *path4end = new TrajectoryAction({{290.0, 35.0, 0.0}, {315.0, 35.0, 0.0}}, l_normConstraints);
        AddAction(new ParallelAction({path4, new IntakeAction(2.9)}));
        // AddAction(new ParallelAction({new SeriesAction({path4, path4end}), new IntakeAction(3.4)}));
        TrajectoryAction *path5 = new TrajectoryAction({{315.0, 40.0, 0.0}, {300.0, 43.0, 0.0}, {192.0, 58.0, 15.0}, {105.0, 138.0, 42.0}}, l_zoomConstraints); // come back and score
        AddAction(new ParallelAction({new SeriesAction({new IntakeAction(0.3), new PrepShooterAction(2.2, 48.5)}), path5}));
        AddAction(new ShootAction(0.7));
    }
};