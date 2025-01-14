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

class FourNoteTRUECenter : public AutoMode
{
public:
    /**
     * @brief 4 note ha ha pl work
     *       
     */
    FourNoteTRUECenter()
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
        TrajectoryAction *path1 = new TrajectoryAction({{55.0, 224.0, 0.0}, {78.0, 224.0, 0.0}}, l_normConstraints);
        AddAction(new SetShootFloor());
        AddAction(new WaitForTimeAction(0.5));
        AddAction(path1);
        AddAction(new ShootAction(1.0));

        // NOTE 2
        TrajectoryAction *path2 = new TrajectoryAction({{78.0, 224.0, 0.0}, {90.0, 224.0, 0.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.3), path2}), new IntakeAction(1.3), new SetOutOfFrameShooting(1.3)}));
        AddAction(new SetShootFloor());
        AddAction(new ShootAction(1.0));

        // NOTE 3
        TrajectoryAction *path3 = new TrajectoryAction({{90.0, 224.0, 0.0}, {193.0, 190.0, 0.0}, {220.0, 170.0, 0.0}, {315.0, 170.0, 0.0}}, l_normConstraints);
        AddAction(new ParallelAction({path3, new SetOutOfFrameShooting(2.6), new SeriesAction({new WaitForTimeAction(0.8), new IntakeAction(1.8)})}));
        TrajectoryAction *path4 = new TrajectoryAction({{315.0, 170.0, 0.0}, {231.0, 176.0, 0.0}, {210.0, 190.0, 0.0}, {90.0, 224.0, 0.0}}, l_normConstraints);
        AddAction(new ParallelAction({new SeriesAction({new IntakeAction(0.5), new SetShootFloor()}), path4, new SetOutOfFrameShooting(0.5)}));
        AddAction(new ShootAction(1.0));

        // 1/2 ;)
        TrajectoryAction *path5 = new TrajectoryAction({{90.0, 224.0, 0.0}, {193.0, 190.0, 0.0}, {220.0, 170.0, 10.0}, {300.0, 120.0, 35.0}}, l_normConstraints);
        AddAction(path5);




        // AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.6), path2}), new IntakeAction(2.0)}));
        // TrajectoryAction *path3 = new TrajectoryAction({{92.0, 169.0, 25.0}, {74.0, 181.0, 15.0}, {74.0, 200.0, 10.0}}, l_normConstraints);
        // AddAction(new ParallelAction({path3, new PrepShooterAction(0.5, 55.0)}));
        // AddAction(new ShootAction(1.0));

        // // NOTE 3
        // TrajectoryAction *path4 = new TrajectoryAction({{74.0, 200.0, 10.0}, {74.0, 224.0, 0.0}, {99.0, 224.0, 0.0}}, l_normConstraints);
        // AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.6), path4}), new IntakeAction(2.1)}));
        // TrajectoryAction *path5 = new TrajectoryAction({{99.0, 224.0, 0.0}, {74.0, 260.0, -25.0}}, l_normConstraints);
        // AddAction(new ParallelAction({new SeriesAction({new IntakeAction(0.7), new PrepShooterAction(0.5, 55.5)}), path5}));
        // AddAction(new ShootAction(1.0));

        // // NOTE 4
        // TrajectoryAction *path6 = new TrajectoryAction({{74.0, 260.0, -25.0}, {108.0, 273.0, -25.0}}, l_normConstraints);
        // AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.6), path6}), new IntakeAction(2.4)}));
        // AddAction(new PrepShooterAction(0.8, 49.87));
        // AddAction(new ShootAction(1.0));
    }
};