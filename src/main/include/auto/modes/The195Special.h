// #pragma once

// #include "auto/modes/AutoMode.h"
// #include "auto/actions/TrajectoryAction.h"
// #include "auto/actions/IntakeAction.h"
// #include "auto/actions/SetShootFloor.h"
// #include "auto/actions/ShootAction.h"
// #include "auto/actions/WaitForTimeAction.h"
// #include "auto/actions/ParallelAction.h"
// #include "auto/actions/SeriesAction.h"

// class The195Special : public AutoMode
// {
// public:
//     /**
//      * @brief Auto mode for the five note auto - 195 special
//      *
//      */
//     The195Special()
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

//         std::vector<double> START_POS = {54.63, 159.95, 0.0};
//         std::vector<double> SHOOT_PRE_1 = {78.1, 187.89, 20.89};
//         std::vector<double> PICKUP_ALIGN_2 = {80.26, 172.73, 19.65};
//         std::vector<double> PICKUP_2 = {89.11, 169.93, 20.56};
//         std::vector<double> SHOOT_2 = {85.85, 218.59, 0.0};
//         std::vector<double> PICKUP_3 = {94.0, 219.06, -0.0};
//         std::vector<double> SHOOT_3 = {85.0, 219.06, -0.0};
//         std::vector<double> PICKUP_ALIGN_4 = {86.7, 256.46, -25.76};
//         std::vector<double> PICKUP_4 = {96.01, 265.3, -30.47};
//         std::vector<double> SHOOT_4 = {80.88, 259.48, -25.02};
//         std::vector<double> PICKUP_ALIGN_5 = {202.24, 278.95, -0.0};
//         std::vector<double> PICKUP_5 = {306.98, 292.87, -0.0};
//         std::vector<double> SHOOT_5 = {90.2, 218.02, -0.0};
//         std::vector<double> PICKUP_ALIGN_1_6 = {187.98, 255.74, -0.0};
//         std::vector<double> PICKUP_ALIGN_2_6 = {244.66, 252.38, -0.0};
//         std::vector<double> PICKUP_ALIGN_3_6 = {289.02, 225.47, -0.0};
//         std::vector<double> PICKUP_6 = {302.99, 225.94, -0.0};
//         std::vector<double> SHOOT_ALIGN_1_6 = PICKUP_ALIGN_2_6;
//         std::vector<double> SHOOT_ALIGN_2_6 = PICKUP_ALIGN_1_6;
//         std::vector<double> SHOOT_6 = {79.43, 238.45, -11.31};

//         TrajectoryAction *SHOOT_1_SEQ = new TrajectoryAction({START_POS, SHOOT_PRE_1}, l_normConstraints);
//         TrajectoryAction *PICKUP_2_SEQ = new TrajectoryAction({SHOOT_PRE_1, PICKUP_ALIGN_2, PICKUP_2}, l_slowConstraints);
//         TrajectoryAction *SHOOT_2_SEQ = new TrajectoryAction({PICKUP_2, SHOOT_2}, l_normConstraints);
//         TrajectoryAction *PICKUP_SHOOT_3_SEQ = new TrajectoryAction({SHOOT_2, PICKUP_3, SHOOT_3}, l_slowConstraints);
//         TrajectoryAction *PICKUP_4_1_SEQ = new TrajectoryAction({SHOOT_3, PICKUP_ALIGN_4}, l_normConstraints);
//         TrajectoryAction *PICKUP_4_2_SEQ = new TrajectoryAction({PICKUP_ALIGN_4, PICKUP_4}, l_slowConstraints);
//         TrajectoryAction *SHOOT_4_SEQ = new TrajectoryAction({PICKUP_4, SHOOT_4}, l_normConstraints);
//         TrajectoryAction *PICKUP_5_SEQ = new TrajectoryAction({SHOOT_4, PICKUP_ALIGN_5, SHOOT_5}, l_normConstraints);
//         TrajectoryAction *PICKUP_6_1_SEQ = new TrajectoryAction({SHOOT_5, PICKUP_ALIGN_1_6, PICKUP_ALIGN_2_6, PICKUP_ALIGN_3_6}, l_normConstraints);
//         TrajectoryAction *PICKUP_6_2_SEQ = new TrajectoryAction({PICKUP_ALIGN_3_6, PICKUP_6}, l_slowConstraints);
//         TrajectoryAction *SHOOT_6_SEQ = new TrajectoryAction({PICKUP_6, SHOOT_ALIGN_1_6, SHOOT_ALIGN_2_6, SHOOT_6}, l_normConstraints);

//         std::
//             vector<Action *>
//                 path = {SHOOT_1_SEQ, new WaitForTimeAction(0.5),
//                         PICKUP_SHOOT_3_SEQ, new WaitForTimeAction(0.5),
//                         PICKUP_4_1_SEQ, PICKUP_4_2_SEQ, SHOOT_4_SEQ, new WaitForTimeAction(0.5),
//                         PICKUP_5_SEQ, new WaitForTimeAction(0.5),
//                         PICKUP_6_1_SEQ, PICKUP_6_2_SEQ, SHOOT_6_SEQ, new WaitForTimeAction(0.5)};

//         std::vector<Action *> sequence = {
//             new SetShootFloor(),
//             new WaitForTimeAction(0.2),
//             new ShootAction(0.5),
//             new IntakeAction(0.5),
//             new SetShootFloor(),
//             new WaitForTimeAction(0.2),
//             new ShootAction(0.5),
//             new IntakeAction(0.2),
//             new SetShootFloor(),
//             new WaitForTimeAction(0.2),
//             new ShootAction(0.5),
//             new IntakeAction(0.5),
//             new SetShootFloor(),
//             new WaitForTimeAction(0.3),
//             new ShootAction(0.5),
//             new IntakeAction(1.5),
//             new SetShootFloor(),
//             new WaitForTimeAction(0.2),
//             new ShootAction(0.5),
//             new IntakeAction(3.0),
//             new SetShootFloor(),
//             new WaitForTimeAction(0.2),
//             new ShootAction(0.5),
//         };

//         AddAction(new ParallelAction({new SeriesAction(path), new SeriesAction(sequence)}));
//     }
// };