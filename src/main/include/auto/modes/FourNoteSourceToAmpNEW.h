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

class FourNoteSourceToAmpNEW : public AutoMode
{
public:
    /**
     * @brief 4 note ha ha pl work
     *       
     */
    FourNoteSourceToAmpNEW()
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

        //* START POS = center against auto line

        // NOTE 1
        // TrajectoryAction *path1 = new TrajectoryAction({{55.0, 224.0, 0.0}, {81.0, 224.0, 0.0}}, l_normConstraints);
        AddAction(new WaitForTimeAction(0.5));
        AddAction(new PrepShooterAction(0.7, 57.2));
        AddAction(new ShootAction(0.7));

        // NOTE 2
        TrajectoryAction *path1 = new TrajectoryAction({{55.0, 224.0, 0.0}, {90.0, 224.0, 0.0}}, l_normConstraints);
        // AddAction(new ParallelAction({path1, new PrepShooterAction(0.5, 55.0)}));
        // TrajectoryAction *path2 = new TrajectoryAction({{81.0, 224.0, 0.0}, {90.0, 224.0, 0.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.6), path1}), new IntakeAction(2.0)}));
        TrajectoryAction *path2 = new TrajectoryAction({{90.0, 224.0, 0.0}, {82.0, 202.0, 18.0}, {74.0, 181.0, 28.0}}, l_normConstraints);
        AddAction(new ParallelAction({path2, new SeriesAction({new PrepShooterAction(0.7, 55.0), new ShootAction(1.0)})}));

        // NOTE 3
        TrajectoryAction *path3 = new TrajectoryAction({{74.0, 181.0, 28.0}, {93.0, 175.0, 28.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.6), path3}), new IntakeAction(1.8)}));
        TrajectoryAction *path4 = new TrajectoryAction({{93.0, 175.0, 28.0}, {84.0, 218.0, -3.0}, {74.0, 260.0, -25.0}}, l_normConstraints);
        AddAction(new ParallelAction({path4, new SeriesAction({new PrepShooterAction(0.7, 55.0), new ShootAction(1.0)})}));

        // NOTE 4
        TrajectoryAction *path5 = new TrajectoryAction({{74.0, 260.0, -25.0}, {108.0, 273.0, -25.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.6), path5}), new IntakeAction(1.8)}));
        AddAction(new PrepShooterAction(0.8, 50.65));
        AddAction(new ShootAction(1.0));

        // TrajectoryAction *path6 = new TrajectoryAction({{108.0, 273.0, -25.0}, {240.0, 161.0, 0.0}}, l_zoomConstraints);
        // AddAction(path6);
    }
};