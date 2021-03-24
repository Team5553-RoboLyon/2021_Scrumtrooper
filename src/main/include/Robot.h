/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Constants.h>
#include <frc/TimedRobot.h>
#if XBOX_CONTROLLER
#include <frc/XboxController.h>
#else
#include <frc/Joystick.h>
#endif
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <frc/PWMSparkMax.h>
#include <frc/VictorSP.h>
#include <networktables/NetworkTableEntry.h>
#include "lib/CSVLogFile.h"
#include "lib/CustomMaths.h"
#include "lib/NL/NLPid.h"
#include "lib/NL/Characterization/NLMotorCharacterization.h"
#include "Joystick.h"
#if IMU
#include <adi/ADIS16470_IMU.h>
#endif
#include <frc/LinearFilter.h>
#include <frc/PowerDistributionPanel.h>

#include "lib/NL/NLTrajectoryStateSPack.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void DriveOld(double forward, double turn);
  void Drive(double jy, double jx);
  void DriveA(double forward, double turn);
  void DriveB();

private:
  NLPID m_pid;
  NLPID_ERROR m_errorLeft;
  NLPID_ERROR m_errorRight;

  Nf32 m_leftErrorVoltage;
  Nf32 m_rightErrorVoltage;
  Nf32 m_refLeftS;
  Nf32 m_refRightS;
  Nf32 m_prevS;
  Nf32 m_prevK;
  Nf32 m_estimatedAngle;
  Nf32 m_dsLeftWheel;
  Nf32 m_dsRightWheel;
  NLTRAJECTORY_STATE_S_PACK m_trajectoryStatesPack;
  NLTRAJECTORY_STATE_S m_currrentSState;
  NLMOTOR_CHARACTERIZATION m_motorCharacterization[4]; //droite: 0,1 gauche: 2,3

  double m_targetLeftSpeed;
  double m_targetRightSpeed;
  VA m_va_left;
  VA m_va_right;
  VA m_va_max;
  KineticToVoltage m_kv;

  nt::NetworkTableEntry m_PowerEntry, m_logGyro, m_speedY, m_speedX, m_throttle;
  rev::CANSparkMax m_moteurDroite{RIGHT_MASTER_GEARBOX_MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_moteurDroiteFollower{RIGHT_SLAVE_GEARBOX_MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_moteurGauche{LEFT_MASTER_GEARBOX_MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_moteurGaucheFollower{LEFT_SLAVE_GEARBOX_MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless};

  frc::PowerDistributionPanel m_pdp;

  frc::Encoder m_encodeurExterneDroite{2, 3, false, frc::Encoder::k4X};
  frc::Encoder m_encodeurExterneGauche{0, 1, true, frc::Encoder::k4X};

#if IMU
  frc::ADIS16470_IMU m_imu{};
  frc::LinearFilter<double> filterX = frc::LinearFilter<double>::MovingAverage(64);
  frc::LinearFilter<double> filterY = frc::LinearFilter<double>::MovingAverage(64);
#endif

  bool m_isPathFollowing = false;
  double m_ramp = 0;
  double m_time0;

  double init_x;
  double init_y;

  CSVLogFile *m_LogFileDriving;

  double m_turnAdjustFactor = 1;

#if XBOX_CONTROLLER
  frc::XboxController m_driverController{0};
#else
  frc::Joystick m_leftHandController{0};
  frc::Joystick m_rightHandController{1};
#endif
  TalonFX m_leftShooterFalcon{LEFT_SHOOTER_FALCON_CAN_ID};
  TalonFX m_rightShooterFalcon{RIGHT_SHOOTER_FALCON_CAN_ID};
};
