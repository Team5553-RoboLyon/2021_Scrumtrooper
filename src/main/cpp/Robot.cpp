/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Constants.h"
#include "Robot.h"
#include "lib/NL/NLOdometry.h"
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <time.h>
#include <units/units.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)

#define AMAX 7   // Acceleration Max  au PIF .. à définir aux encodeurs
#define VMAX 3.4 // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define JMAX 45
#define WMAX ((2.0 * VMAX) / TRACKWIDTH) // vitesse angulaire Max theorique
#define PATHNAME "0_8Great2UJ30A1_5V1"

#define VOLTAGE_COMPENSATION_VALUE 10

#define TIME_RAMP 0

double Deadband(double value, double deadband = 0.1)
{
    if (std::abs(value) < deadband)
        return 0;
    else
        return value < 0 ? (value + deadband) / (1.0 - deadband) : (value - deadband) / (1.0 - deadband);
}

void Robot::DriveOld(double forward, double turn)
{

    forward = Deadband(forward, 0.1);
    turn = Deadband(turn, 0.2);

    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    m_moteurGauche.Set(lwheel);
    m_moteurGaucheFollower.Set(lwheel);
    m_moteurDroite.Set(rwheel);
    m_moteurDroiteFollower.Set(rwheel);
}

void Robot::Drive(double forward, double turn)
{
    forward = Deadband(forward, 0.1);
    turn = Deadband(turn, 0.1);
    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    double target_left_speed = lwheel * VMAX;
    double target_right_speed = rwheel * VMAX;

    updateVelocityAndAcceleration(&m_va_left, &m_va_max, target_left_speed, 0.02);
    updateVelocityAndAcceleration(&m_va_right, &m_va_max, target_right_speed, 0.02);

    m_moteurGauche.Set(m_kv.getVoltage(0, &m_va_left) / m_moteurGauche.GetBusVoltage());
    m_moteurDroite.Set(m_kv.getVoltage(2, &m_va_right) / m_moteurDroite.GetBusVoltage());
}
void Robot::DriveA(double forward, double turn)
{
    forward = Deadband(forward, 0.1);
    turn = Deadband(turn, 0.15);
    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    m_targetLeftSpeed = lwheel * VMAX;
    m_targetRightSpeed = rwheel * VMAX;
}
void Robot::DriveB()
{
    updateVelocityAndAcceleration(&m_va_left, &m_va_max, m_targetLeftSpeed, 0.005);
    updateVelocityAndAcceleration(&m_va_right, &m_va_max, m_targetRightSpeed, 0.005);

    m_moteurGauche.Set(m_kv.getVoltage(0, &m_va_left) / m_moteurGauche.GetBusVoltage());
    m_moteurGaucheFollower.Set(m_kv.getVoltage(1, &m_va_left) / m_moteurGaucheFollower.GetBusVoltage());
    m_moteurDroite.Set(m_kv.getVoltage(2, &m_va_right) / m_moteurDroite.GetBusVoltage());
    m_moteurDroiteFollower.Set(m_kv.getVoltage(3, &m_va_right) / m_moteurDroiteFollower.GetBusVoltage());
}

void Robot::RobotInit()
{
    m_moteurDroite.RestoreFactoryDefaults();
    m_moteurGauche.RestoreFactoryDefaults();
    m_moteurDroiteFollower.RestoreFactoryDefaults();
    m_moteurGaucheFollower.RestoreFactoryDefaults();

    m_moteurDroite.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGauche.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurDroiteFollower.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGaucheFollower.SetOpenLoopRampRate(TIME_RAMP);

    m_moteurDroite.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGauche.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurDroiteFollower.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGaucheFollower.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);

    m_moteurGauche.DisableVoltageCompensation();
    m_moteurGaucheFollower.DisableVoltageCompensation();
    m_moteurDroite.DisableVoltageCompensation();
    m_moteurDroiteFollower.DisableVoltageCompensation();
    m_moteurDroite.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurGauche.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurDroiteFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurGaucheFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_PowerEntry = frc::Shuffleboard::GetTab("voltage").Add("Voltage", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#if IMU
    m_speedY = frc::Shuffleboard::GetTab("voltage").Add("speedY", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_speedX = frc::Shuffleboard::GetTab("voltage").Add("speedX", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#endif
    m_encodeurExterneDroite.SetReverseDirection(true);
    m_encodeurExterneGauche.SetReverseDirection(false);

    m_encodeurExterneDroite.SetDistancePerPulse(1);
    m_encodeurExterneGauche.SetDistancePerPulse(1);

    m_encodeurExterneDroite.SetSamplesToAverage(65);
    m_encodeurExterneGauche.SetSamplesToAverage(65);

    m_moteurDroite.SetInverted(false);
    m_moteurDroiteFollower.SetInverted(false);

    m_moteurGauche.SetInverted(true);
    m_moteurGaucheFollower.SetInverted(true);

    //Left 1 Forward
    m_kv.SetMotorCoefficients(0, 0, 3.1326206010537563, 0.5677475451077377, 0.13130778674420807);
    //Left 2 Forward
    m_kv.SetMotorCoefficients(1, 0, 3.1235193481564174, 0.5556044575328529, 0.14299467623195827);
    //Right 1 Forward
    m_kv.SetMotorCoefficients(2, 0, 3.098541009252153, 0.3552462306731122, 0.1591323786822345);
    //Right 2 Forward
    m_kv.SetMotorCoefficients(3, 0, 3.097982066096822, 0.3571148418248107, 0.16019545143144676);
    //Left 1 Backward
    m_kv.SetMotorCoefficients(0, 1, 3.161121333666647, 0.5486387690059814, -0.13693734149024817);
    //Left 2 Backward
    m_kv.SetMotorCoefficients(1, 1, 3.151959813222301, 0.5385715415354247, -0.14931540125265652);
    //Right 1 Backward
    m_kv.SetMotorCoefficients(2, 1, 3.0649666485836486, 0.4261078385729976, -0.15383003389418626);
    //Right 2 Backward
    m_kv.SetMotorCoefficients(3, 1, 3.064038233186117, 0.4359525131849489, -0.15504809860349766);

    m_va_max.m_speed = VMAX;
    m_va_max.m_acceleration = AMAX;
    m_va_max.m_jerk = JMAX;

    m_va_left.m_speed = 0;
    m_va_left.m_acceleration = 0;
    m_va_right.m_speed = 0;
    m_va_right.m_acceleration = 0;

    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);
    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);
    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);
    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);
    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    Robot::AddPeriodic([&]() {
        if (m_isPathFollowing)
        {
            // *****************************************************    'THE' METHOD(e)
            // feed back:
            // avec les encodeurs on estime la position du robot:
            //			l = distance parcourue par la roue gauche depuis le dernier reset encodeur.
            //			r = distance parcourue par la roue droite depuis le dernier reset encodeur.
            Nf32 l = (m_encodeurExterneGauche.GetDistance() * NF32_2PI * WHEEL_RADIUS) / 2048.0f;
            Nf32 r = (m_encodeurExterneDroite.GetDistance() * NF32_2PI * WHEEL_RADIUS) / 2048.0f;

            //			dl et dr = distances parcourues par les roues gauche et droite depuis le dernier call.
            //			(note dl/dt = vitesse roue gauche et dr/dt = vitesse roue droite droite )
            Nf32 dl = l - m_dsLeftWheel;
            Nf32 dr = r - m_dsRightWheel;

            // mise à jour des distances totales parcourues par chaque roue.
            m_dsLeftWheel = l;
            m_dsRightWheel = r;

            // mise à jour de la position et de l'angle "estimés" du robot.
            Nf32 v = NLODOMETRY_DRIVETRAIN_V_FROM_WHEELS(dl, dr);
            Nf32 w = NLODOMETRY_DRIVETRAIN_W_FROM_WHEELS(dl, dr, TRACKWIDTH);

            // méthode simplifiée pour de petites valeurs de w où on considère que le déplacement du robot est un petit segment de droite
            m_estimatedAngle += w;
            m_currrentSState.m_kin.m_t += 0.005f;
            m_trajectoryStatesPack.getState(&m_currrentSState, m_currrentSState.m_kin.m_t);

            Nf32 ds = m_currrentSState.m_kin.m_s - m_prevS;
            Nf32 k = (m_currrentSState.m_localCurvature + m_prevK) / 2.0f;
            if (k != 0.0f)
            {
                Nf32 radius = 1.0f / k;
                m_refLeftS += ds * (radius - HALF_TRACKWIDTH) / radius;
                m_refRightS += ds * (radius + HALF_TRACKWIDTH) / radius;
            }
            else
            {
                m_refLeftS += ds;
                m_refRightS += ds;
            }
            m_prevK = m_currrentSState.m_localCurvature;
            m_prevS = m_currrentSState.m_kin.m_s;

            m_errorLeft.update(m_refLeftS - m_dsLeftWheel);
            m_errorRight.update(m_refRightS - m_dsRightWheel);

            m_leftErrorVoltage = m_pid.command(&m_errorLeft);
            m_rightErrorVoltage = m_pid.command(&m_errorRight);

            // 2) avec k on a R = 1/k et avec R on a la distribution gauche droite
            if (m_currrentSState.m_localCurvature == 0.0f)
            {
                m_moteurGauche.SetVoltage(units::volt_t(m_motorCharacterization[2].getVoltage(m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a) + m_leftErrorVoltage));
                m_moteurGaucheFollower.SetVoltage(units::volt_t(m_motorCharacterization[3].getVoltage(m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a) + m_leftErrorVoltage));
                m_moteurDroite.SetVoltage(units::volt_t(m_motorCharacterization[0].getVoltage(m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a) + m_rightErrorVoltage));
                m_moteurDroiteFollower.SetVoltage(units::volt_t(m_motorCharacterization[1].getVoltage(m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a) + m_rightErrorVoltage));
                m_LogFileDriving->Log(m_encodeurExterneGauche.GetDistance(), m_encodeurExterneDroite.GetDistance(), m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a, m_currrentSState.m_kin.m_a, m_leftErrorVoltage, m_rightErrorVoltage);
            }
            else
            {

                r = 1.0f / m_currrentSState.m_localCurvature;
                Nf32 left = (r - HALF_TRACKWIDTH) * m_currrentSState.m_localCurvature;
                Nf32 right = (r + HALF_TRACKWIDTH) * m_currrentSState.m_localCurvature;

                m_moteurGauche.SetVoltage(units::volt_t(m_motorCharacterization[2].getVoltage(m_currrentSState.m_kin.m_v * left, m_currrentSState.m_kin.m_a * left) + m_leftErrorVoltage));
                m_moteurGaucheFollower.SetVoltage(units::volt_t(m_motorCharacterization[3].getVoltage(m_currrentSState.m_kin.m_v * left, m_currrentSState.m_kin.m_a * left) + m_leftErrorVoltage));
                m_moteurDroite.SetVoltage(units::volt_t(m_motorCharacterization[0].getVoltage(m_currrentSState.m_kin.m_v * right, m_currrentSState.m_kin.m_a * right) + m_rightErrorVoltage));
                m_moteurDroiteFollower.SetVoltage(units::volt_t(m_motorCharacterization[1].getVoltage(m_currrentSState.m_kin.m_v * right, m_currrentSState.m_kin.m_a * right) + m_rightErrorVoltage));
                m_LogFileDriving->Log(m_encodeurExterneGauche.GetDistance(), m_encodeurExterneDroite.GetDistance(), m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_v * left, m_currrentSState.m_kin.m_v * right, m_currrentSState.m_kin.m_a * left, m_currrentSState.m_kin.m_a * right, m_leftErrorVoltage, m_rightErrorVoltage);
            }
        }
        else
        {
            DriveB();
        }
    },
                       5_ms, 5_ms);

    // TRAJECTORY
    wpi::SmallString<64> deployDirectory;
    frc::filesystem::GetDeployDirectory(deployDirectory);
    char name[64];
    sprintf(name, "%s.tsp", PATHNAME);
    wpi::sys::path::append(deployDirectory, name);
    std::cout << deployDirectory << std::endl;

    std::cout << "file open to read" << std::endl;
    FILE *pfile = fopen(deployDirectory.c_str(), "rb");
    m_trajectoryStatesPack.read(pfile);
    fclose(pfile);
    std::cout << "file close" << std::endl;

    m_pid.m_kP = 60.0f;
    m_pid.m_kI = 0.0f;
    m_pid.m_kD = 0.0f;

    m_motorCharacterization[0].setForwardConst(3.1326206010537563, 0.5677475451077377, 0.13130778674420807);
    m_motorCharacterization[1].setForwardConst(3.1235193481564174, 0.5556044575328529, 0.14299467623195827);
    m_motorCharacterization[2].setForwardConst(3.098541009252153, 0.3552462306731122, 0.1591323786822345);
    m_motorCharacterization[3].setForwardConst(3.097982066096822, 0.3571148418248107, 0.16019545143144676);

    m_motorCharacterization[0].setBackwardConst(3.161121333666647, 0.5486387690059814, -0.13693734149024817);
    m_motorCharacterization[1].setBackwardConst(3.151959813222301, 0.5385715415354247, -0.14931540125265652);
    m_motorCharacterization[2].setBackwardConst(3.0649666485836486, 0.4261078385729976, -0.15383003389418626);
    m_motorCharacterization[3].setBackwardConst(3.064038233186117, 0.4359525131849489, -0.15504809860349766);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    m_encodeurExterneDroite.Reset();
    m_encodeurExterneGauche.Reset();

    m_isPathFollowing = 0;

    // reset robot path
    m_dsLeftWheel = 0.0f;
    m_dsRightWheel = 0.0f;

    m_currrentSState.null();

    m_refLeftS = 0.0f;
    m_refRightS = 0.0f;
    m_prevK = 0.0f;
    m_prevS = 0.0f;

    m_errorLeft.reset();
    m_errorRight.reset();

#if IMU
    init_x = m_imu.GetAccelInstantX();
    init_y = m_imu.GetAccelInstantY();
#endif
}

void Robot::TeleopPeriodic()
{
#if IMU
    m_speedY.SetDouble(filterY.Calculate(m_imu.GetAccelInstantY() - init_y));
    m_speedX.SetDouble(filterX.Calculate(m_imu.GetAccelInstantX() - init_x));
#endif

#if XBOX_CONTROLLER
    DriveA(-m_driverController.GetY(frc::GenericHID::JoystickHand::kLeftHand), m_driverController.GetX(frc::GenericHID::JoystickHand::kRightHand));
#else:
    m_va_max.m_acceleration = m_PowerEntry.GetDouble(0.0f);
    DriveA(-m_leftHandController.GetY(), m_rightHandController.GetZ());
    std::cout << m_encodeurExterneGauche.GetDistance() << std::endl;
#endif
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
