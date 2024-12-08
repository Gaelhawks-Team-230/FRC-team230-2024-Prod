#pragma once

#include "frc/smartdashboard/SmartDashboard.h"
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <units/time.h>

#include "Constants.h"
#include "util/Subsystem.h"
#include "util/MathUtil.h"
#include "util/planners/Interpolator.h"

// TODO: DOES NOT WORK, NEED TO FIX
const int NOTE_OBJECT_DETECTION_PIPELINE = 0;

const double KXY = 2.0;
const double KPSI = 4.0;

const double CAMERA_HEADING_OFFSET = -5.0;

const double XY_VEL_LIMIT = 75.0; // inches per second
const double R_VEL_LIMIT = 80.0;  // degrees per second

// Goal pitch of Note deg
const double GOAL_RY = -16.0;
// Goal yaw of Note deg
const double GOAL_RZ = 0.0;

const std::map<double, std::vector<double>> NOTE_DISTANCE_TABLE{{-14.0, {6}}, {-12.2, {12}}, {-9.6, {24}}, {-7.5, {36}}, {-6.2, {48}}, {-5.7, {60}}, {-5.0, {72}}, {-4.6, {84}}, {-4.0, {96}}, {-3.8, {108}}};
// const std::map<double, std::vector<double>> NOTE_YAW_TABLE{{, {}}, {, {}}, {, {}}, {, {}}, {, {}}, {, {}}, {, {}}, {, {}}, {, {}}, {, {}}, {, {}}, {, {}}, {, {}}};

const std::string NOTE_CAMERA_NAME = "NoteCamera";

class NoteVision: public Subsystem
{
public:
    void LocalReset(void);
    void Analyze();
    void UpdateDash();
    void Periodic(){};
    void Stop(){};
    void DriveTargetting(double *xdot, double *ydot);
    bool CheckNotePosition(double min, double max);
    void SetGoalXDist(double dist) { m_goalXDist = dist; };
    double GetGoalXDist() { return m_goalXDist; };

    static NoteVision *GetInstance();

private:
    NoteVision();
    static NoteVision *m_instance;

    nt::BooleanSubscriber m_hasTargetSub;
    nt::DoubleSubscriber m_latencySub;
    nt::DoubleSubscriber m_yawSub;
    nt::DoubleSubscriber m_pitchSub;

    nt::IntegerPublisher m_piplinePub;

    bool m_hasTarget;
    units::time::second_t m_latency;
    double m_yaw;
    double m_pitch;
    double m_goalXDist;
    double m_xDist;
    // double m_headingOffset;

    void SetPipeline(int pipeline)
    {
        m_piplinePub.Set(pipeline);
    }

    Interpolator *m_noteTable;
    // Interpolator *m_noteYawTable;
};