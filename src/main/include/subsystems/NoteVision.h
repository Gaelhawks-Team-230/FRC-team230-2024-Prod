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

// TODO DOES NOT WORK, NEED TO FIX
const int NOTE_OBJECT_DETECTION_PIPELINE = 0;

const double KXY = 4.0;
const double KPSI = 3.0;

const double CAMERA_HEADING_OFFSET = 0.0;

const double XY_VEL_LIMIT = 100.0; // inches per second
const double R_VEL_LIMIT = 180.0;  // degrees per second

// * Pitch of camera, distance from robot
const std::map<double, std::vector<double>> NOTE_DISTANCE_TABLE{{-20.0, {0.0}},{-16.0, {8.0}}, {-15.3, {12}}, {-7.4, {24}}, {-2.6, {36}}, {1.0, {48}}, {3.5, {60}}, {5.5, {72}}};
const std::map<double, std::vector<double>> NOTE_YAW_TABLE{{0.0, {-4.11}},{8.0 , {0.35}}, {12, { 2.4}}, { 24, {7.0}}, {36.0, {9}}, {48, {10.9}}, {60, {12.1}}, {72, {12.9}}};

const std::string NOTE_CAMERA_NAME = "NoteCamera";

class NoteVision : public Subsystem
{
public:
    void LocalReset(void);
    void Analyze();
    void UpdateDash();
    void Periodic(){};
    void Stop(){};
    void DriveControl(double &xdot, double &ydot, double &psidot);
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
    double m_heading;

    void SetPipeline(int pipeline)
    {
        m_piplinePub.Set(pipeline);
    }

    Interpolator *m_noteTable;
    Interpolator *m_noteYawTable;
};