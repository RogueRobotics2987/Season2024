// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "../cpp/CommandMessenger.cpp"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LimelightSubsystem.h"
#include "networktables/NetworkTableInstance.inc"

#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveStateMachine
    : public frc2::CommandHelper<frc2::Command, DriveStateMachine> {
 public:
  DriveStateMachine();
  DriveStateMachine(DriveSubsystem &drive, LimelightSubsystem &limelight, frc::XboxController &driveXbox, frc::XboxController &auxXbox, CommandMessenger &messager);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  float Deadzone(float x);

 private:
  enum driveState{NONE, NOTE_FOLLOW, APRIL_FOLLOW};
  driveState drive_state = NONE;

  DriveSubsystem* m_drive;
  LimelightSubsystem* m_limelight;
  CommandMessenger* m_messager;

  frc::XboxController* m_driverController = nullptr;
  frc::XboxController* m_auxController = nullptr;


  double speedX = 0;
  double speedY = 0;
  double rot = 0.0;
  bool NoJoystickInput = false;

  units::angular_velocity::radians_per_second_t rotNote = units::angular_velocity::radians_per_second_t(0);
  units::velocity::meters_per_second_t speedNote = units::velocity::meters_per_second_t(0);
  double kpNote = 0.09927;
  double txNote = 0.0;

  units::angular_velocity::radians_per_second_t rotApril = units::angular_velocity::radians_per_second_t(0);
  double kpApril = 0.02206;
  double txApril = 0.0;


  bool noteFollowState = false;
  bool aprilFollowState = false;
  bool standard = false;

  bool runIntake = false;
  bool runShooterWarmup = false;

};
