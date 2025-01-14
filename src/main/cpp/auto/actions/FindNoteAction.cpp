// #include "auto/actions/FindNoteAction.h"

// FindNoteAction::FindNoteAction(double minDetectDist, double maxDetectDist) : m_minDetectDist(minDetectDist), m_maxDetectDist(maxDetectDist)
// {
//     // m_autonomous = Autonomous::GetInstance();
//     m_drivetrain = Drivetrain::GetInstance();
//     m_noteVision = NoteVision::GetInstance();
// }

// void FindNoteAction::Periodic()
// {
//     double xdot, ydot, psidot;

//     // xdot = m_autonomous->GetAutoXDot();
//     // ydot = m_autonomous->GetAutoYDot();
//     // psidot = m_autonomous->GetAutoPsidot();

//     m_noteVision->DriveTargetting(&xdot, &psidot);
//     // printf("%f, %f, %f\n", xdot, ydot, psidot);
//     m_drivetrain->SetChassisSpeeds(xdot, 0.0, psidot);
// }

// bool FindNoteAction::IsActionComplete()
// {
//     return (!m_noteVision->CheckNotePosition(m_minDetectDist, m_maxDetectDist));
// }