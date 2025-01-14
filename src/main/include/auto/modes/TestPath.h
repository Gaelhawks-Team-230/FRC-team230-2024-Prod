// #pragma once

// #include <frc/DriverStation.h>
// #include "auto/modes/AutoMode.h"
// #include "auto/actions/TrajectoryAction.h"
// #include "auto/actions/PrepShooterAction.h"
// #include "auto/actions/ShootAction.h"
// #include "auto/actions/SeriesAction.h"
// #include "auto/actions/ParallelAction.h"
// #include "auto/actions/WaitForTimeAction.h"
// #include "auto/actions/IntakeAction.h"
// #include "auto/actions/StowArmAction.h"



// class TestPath : public AutoMode
// {
// public:
//     /**
//      * @brief Test path for testing purposes.
//      * 
//      */
//     TestPath()
//     {
//         Tconstraints l_constraints;
//         l_constraints.amax = 40.0;
//         l_constraints.vmax = 75.0;
//         l_constraints.vmin = 0.0;

//         SetInitialRobotHeading(0.0);


//         AddAction(new IntakeAction(5.0));
//         AddAction(new ParallelAction({new SeriesAction({new WaitForTimeAction(2.0), new ShootAction(2.0)}), new PrepShooterAction(5.0, 10.0)}));
//         AddAction(new StowArmAction());
//     }
// };

// // ! TESTING OBSERVATIONS
// //* xdot cmds should be pos BOTH WAYS
// //* ydot cmds should be pos for blue, neg for red
// //* psidot cmds should be pos for blue, neg for red