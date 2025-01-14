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

class ThreeNoteCenterToSource : public AutoMode
{
public:
    /**
     * @brief 4 note ha ha pl work
     *       
     */
    ThreeNoteCenterToSource()
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
        // initial trajectory ripped from 3 1/2
        // TrajectoryAction *path3 = new TrajectoryAction({{90.0, 224.0, 0.0}, {75.0, 210.0, 5.0}, {75.0, 200.0, 15.0}, {105.0, 181.0, 25.0}}, l_normConstraints);
        // possible trajectory based on classic 4 note
        TrajectoryAction *path3 = new TrajectoryAction({{90.0, 224.0, 0.0}, {75.0, 210.0, 5.0}, {75.0, 200.0, 15.0}, {96.0, 173.0, 25.0}}, l_normConstraints);

        AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(0.3), path3}), new IntakeAction(1.8), new SetOutOfFrameShooting(1.8)}));
        AddAction(new SetShootFloor());
        AddAction(new ShootAction(1.0));
    }
};