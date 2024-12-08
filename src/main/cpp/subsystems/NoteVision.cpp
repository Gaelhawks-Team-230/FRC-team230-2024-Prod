#include "subsystems/NoteVision.h"

NoteVision *NoteVision::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new NoteVision();
    }
    return m_instance;
}
NoteVision::NoteVision()
{
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    auto datatable = inst.GetTable("photonvision");

    m_hasTargetSub = datatable->GetBooleanTopic("NoteCamera/hasTarget").Subscribe(false);
    m_latencySub = datatable->GetDoubleTopic("NoteCamera/latencyMillis").Subscribe(0.0);
    m_yawSub = datatable->GetDoubleTopic("NoteCamera/targetYaw").Subscribe(0.0);
    m_pitchSub = datatable->GetDoubleTopic("NoteCamera/targetPitch").Subscribe(0.0);
    m_piplinePub = datatable->GetIntegerTopic("NoteCamera/pipelineIndex").Publish();

    m_noteTable = new Interpolator(NOTE_DISTANCE_TABLE);
    // m_noteYawTable = new Interpolator(NOTE_YAW_TABLE);

    LocalReset();
}

void NoteVision::Analyze()
{
    m_hasTarget = m_hasTargetSub.Get();
    m_latency = units::time::millisecond_t{m_latencySub.Get()};
    m_yaw = m_yawSub.Get();
    m_pitch = m_pitchSub.Get();
    m_xDist = m_noteTable->Sample(m_pitch)[0];
    // m_headingOffset = m_noteTable->Sample(m_yaw)[0];
}

void NoteVision::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Vision/Note/PipelineLatency", m_latency.value());
    frc::SmartDashboard::PutBoolean("Vision/Note/HasTarget", m_hasTarget);
    frc::SmartDashboard::PutNumber("Vision/Note/Yaw", m_yaw);
    frc::SmartDashboard::PutNumber("Vision/Note/Pitch", m_pitch);
}
void NoteVision::LocalReset()
{
    SetPipeline(NOTE_OBJECT_DETECTION_PIPELINE);
    m_hasTarget = false;
    m_latency = units::time::second_t{0};
    m_pitch = 0.0;
    m_yaw = 0.0;
    m_goalXDist = 0.0;
}
bool NoteVision::CheckNotePosition(double min, double max)
{
    if (!m_hasTarget)
    {
        return false;
    }

    return (m_xDist > min && m_xDist < max);

}
void NoteVision::DriveTargetting(double *xdot, double *psidot)
{
    double vxdot, vpsidot;

    if (!m_hasTarget)
    {
        return;
    }

    vxdot = -(KXY) * (m_xDist - m_goalXDist);
    vpsidot = (KPSI) * (m_yaw - CAMERA_HEADING_OFFSET);

    *xdot += MathUtil::Limit(-XY_VEL_LIMIT, XY_VEL_LIMIT, vxdot);
    *psidot += MathUtil::Limit(-R_VEL_LIMIT, R_VEL_LIMIT, vpsidot);
}

NoteVision *NoteVision::m_instance = nullptr;