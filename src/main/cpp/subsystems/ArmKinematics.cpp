#include "subsystems/ArmKinematics.h"
using json = nlohmann::json;
using namespace std;

ArmKinematics *ArmKinematics::GetInstance()
{
    if (m_armK == nullptr)
    {
        m_armK = new ArmKinematics();
    }
    return m_armK;
}

ArmKinematics::ArmKinematics()
{
    m_arm = Arm::GetInstance();
    CreateMap();
    m_interpol = new Interpolator(ArmInterpolationTable);
    m_goalPos = 30.0;
    m_nextPos = 30.0;
}

/**
 * @brief This reads in a JSON object and fills in the values to the map
 *
 * @param objects This is the JSON object that is getting read in
 * @return map<double, vector<double>>
 */
map<double, vector<double>> ArmKinematics::CreateMap()
{
    string directory = frc::filesystem::GetDeployDirectory();

    ifstream f(directory + "/arm_pos_table.json");
    json j = json::parse(f);
    auto data = j;

    for (auto &item : data)
    {
        keyVal = item["index"].get<double>();

        auto val = item["values"].get<vector<double>>();
        ArmInterpolationTable.insert({keyVal, val});
    }
    return ArmInterpolationTable;
}

void ArmKinematics::GoToPickup()
{
    m_goalPos = 0.0; // TODO: Change after testing
}
/**
 * @brief Commands the arm to move to the stow position
 *
 */
void ArmKinematics::GoToStow()
{
    m_goalPos = 30.0; // TODO: Change after testing
}
/**
 * @brief This commands the arm and platform to move to the interpolated values from the shooter solution
 *
 * @param p_shooterPercentage This is the index that the shooter solution provides for the arm interpolation table to interpolate off of. SHOULD BE 40 -> 60!!
 */
void ArmKinematics::GoToShootingPos(double p_shooterPercentage)
{
    m_goalPos = p_shooterPercentage;
}
/**
 * @brief Commands arm to move to the amp position
 *
 */
void ArmKinematics::GoToAmp()
{
    m_goalPos = 80.0; // TODO: Change after testing
}
/**
 * @brief Commands arm to move to the trap position
 *
 */
void ArmKinematics::GoToHighTrap()
{
    m_goalPos = 100.0; // TODO: Change after testing
}
/**
 * @brief Commands arm to move to the trap position
 *
 */
void ArmKinematics::GoToLowTrap()
{
    m_goalPos = 90.0; // TODO: Change after testing
}
/**
 * @brief This function will provide the arm's control systems (platform/arm) with new position goals.
 *
 */
void ArmKinematics::Update()
{
    m_goalPos = MathUtil::Limit(0, 100, m_goalPos);
    // local variables that should NEVER be used somewhere else
    double prate;
    double nrate;

    prate = rateTable.at(m_interpol->FindLowerKey(ArmInterpolationTable, m_nextPos));
    nrate = prate * -1.0;
    m_nextPos = m_nextPos + MathUtil::Limit(nrate * Constants::LOOPTIME, prate * Constants::LOOPTIME, m_goalPos - m_nextPos);

    vector<double> resultsAtNextPos = m_interpol->Sample(m_nextPos);
    m_arm->SetPosition(resultsAtNextPos.at(0), resultsAtNextPos.at(1));
}
ArmKinematics *ArmKinematics::m_armK = nullptr;
