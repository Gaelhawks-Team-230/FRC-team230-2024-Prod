#include "RobotState.h"

RobotState *RobotState::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new RobotState();
    }
    return m_instance;
}

RobotState::RobotState()
{
    Reset(Pose2d());
}

/**
 * @brief Updates the robot's pose based on velocities and odometry.
 *
 * @param vx Linear velocity in the x direction (in/s).
 * @param vy Linear velocity in the y direction (in/s).
 * @param psidot Rotational velocity (deg/s).
 */
std::tuple<double, double, double> RobotState::IntegrateOdomotryVel(double vx, double vy, double vpsi, double heading)
{

    // Create a 2x2 rotation matrix based on the current orientation
    double rotation_matrix[2][2] = {{MathUtil::cosd(heading), -MathUtil::sind(heading)},
                                    {MathUtil::sind(heading), MathUtil::cosd(heading)}};

    // Calculate the displacement vector in the x and y directions
    double displacement[2] = {vx * Constants::LOOPTIME, vy * Constants::LOOPTIME};

    // Rotate the displacement vector using the rotation matrix
    double rotated_displacement[2] = {
        rotation_matrix[0][0] * displacement[0] + rotation_matrix[0][1] * displacement[1],
        rotation_matrix[1][0] * displacement[0] + rotation_matrix[1][1] * displacement[1]};

    // Update the robot's position (x, y) by adding the rotated displacement
    // Update the robot's orientation (theta) by rotating it based on psi and dt

    // Update the robot's orientation (theta) by using the gyro heading
    // theta = gyroHeading;

    return std::make_tuple(rotated_displacement[0], rotated_displacement[1], vpsi * Constants::LOOPTIME);
}
/**
 * @brief Update the robot's pose based on the odometry and vision data.
 *
 * @param p_visionPose optional vision pose
 * @param vx velocity in the x direction being send to drivetrain
 * @param vy velocity in the y direction being send to drivetrain
 * @param vpsi rotational velocity being send to drivetrain
 */
void RobotState::Update(std::optional<Pose2d> p_visionPose, double vx, double vy, double vpsi)
{
    Pose2d updatedOdometryPose, updatedVisionPose;

    double dx, dy, drotation;
    double x, y, rotation;
    x = m_pose.GetX();
    y = m_pose.GetY();
    rotation = m_pose.GetRotation();

    std::tie(dx, dy, drotation) = IntegrateOdomotryVel(vx, vy, vpsi, rotation);
    updatedOdometryPose = Pose2d(x + dx, y + dy, MathUtil::Wrap(rotation + drotation));

    if (p_visionPose.has_value())
    {
        updatedVisionPose = p_visionPose.value();
        // Use complimentary filtering to combine the odometry and vision poses
        x = m_xFilter.HighPassFilter(updatedOdometryPose.GetX()) + m_xFilter.LowPassFilter(updatedVisionPose.GetX());
        y = m_yFilter.HighPassFilter(updatedOdometryPose.GetY()) + m_yFilter.LowPassFilter(updatedVisionPose.GetY());
        rotation = m_rotationFilter.HighPassFilter(updatedOdometryPose.GetRotation()) + m_rotationFilter.LowPassFilter(updatedVisionPose.GetRotation());
        m_pose = Pose2d(x, y, rotation);
    }
    else
    {
        m_pose = updatedOdometryPose;
    }
}

RobotState *RobotState::m_instance = nullptr;