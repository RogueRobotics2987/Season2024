// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "../cpp/CommandMessenger.cpp"
#include "MessengerCommand.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LimelightSubsystem.h"
#include "networktables/NetworkTableInstance.inc"
#include "../cpp/AutoPaths.cpp"

#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <list>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoDriveStateMachine
    : public frc2::CommandHelper<frc2::Command, AutoDriveStateMachine> {
 public:
  AutoDriveStateMachine();
  AutoDriveStateMachine(DriveSubsystem &drive, LimelightSubsystem &limelight, 
    frc::XboxController &driveXbox, frc::XboxController &auxXbox, MessengerCommand &message, /*CommandMessenger &messager,*/ 
    std::vector<AutoPaths::AutoPath> &path
  );

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  float Deadzone(float x);

  double DistanceBetweenAngles(double targetAngle, double sourceAngle);


 private:
  enum driveState{NONE, NOTE_FOLLOW, APRIL_FOLLOW, PRE_PATH_FOLLOW, PATH_FOLLOW};
  driveState drive_state = NONE;

  DriveSubsystem* m_drive;
  LimelightSubsystem* m_limelight;
  //CommandMessenger* m_messager;
  MessengerCommand* m_messager;

  frc::XboxController* m_driverController = nullptr;
  frc::XboxController* m_auxController = nullptr;


  units::angular_velocity::radians_per_second_t rotNote = units::angular_velocity::radians_per_second_t(0);
  units::velocity::meters_per_second_t speedNote = units::velocity::meters_per_second_t(0);
  double kpNote = 0.09927;
  double txNote = 0.0;

  units::angular_velocity::radians_per_second_t rotApril = units::angular_velocity::radians_per_second_t(0);
  // double kpApril = 0.02206;
  double kpApril = 0.005;
  double speedX = 0;
  double speedY = 0;
  bool NoJoystickInput = false;
  double txApril = 0.0;


  bool noteFollowState = false;
  bool aprilFollowState = false;
  bool standard = false;
  bool pathFollowState = false;

  bool runIntake = false;
  bool runShooterWarmup = false;

  
    std::list<frc::Pose2d> m_waypoints;
    std::list<units::meters_per_second_t> m_driveSpeed;
    std::list<units::meters_per_second_t> m_cruiseSpeed;
    std::list<std::string> m_command;
    std::list<bool> m_limefollowAuto;
    std::list<double> m_waypointDistance;
    std::list<AutoPaths::AutoPath> paths;


    bool finished = false;
    frc::Pose2d currentPose;
    frc::Pose2d desiredPose;
    units::meters_per_second_t robotSpeed;
    double alpha = 0; // Possibly change to rotation?
    units::meters_per_second_t xVal;
    units::meters_per_second_t yVal;
    units::radians_per_second_t thetaVal;
    double totalDistance = 0;
    double deltaX = 0;
    double deltaY = 0;
    double distanceTraveled = 0;
    double thetaDouble = 0;
    frc::Pose2d lastPose;
    units::meters_per_second_t maxSpeed;
    double threshold = 0.1;
    units::meters_per_second_t pointSpeed;
    units::meters_per_second_t cruiseSpeed;
    double currentDistance = 0;
    bool apriltagBool = false;
    std::string command;
    double accumulatedError = 0;
    units::meters_per_second_t lastPointSpeed = 0_mps;

};
