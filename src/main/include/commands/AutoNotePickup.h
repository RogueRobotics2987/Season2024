// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "networktables/NetworkTableInstance.inc"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LimelightSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
 
class AutoNotePickup : public frc2::CommandHelper<frc2::Command, AutoNotePickup>
{
  public:
    AutoNotePickup();
    AutoNotePickup(
      LimelightSubsystem &limePose,
      DriveSubsystem &drivetrain,
      IntakeSubsystem &intake
      );

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

  private:
    LimelightSubsystem* m_limelight = nullptr;
    DriveSubsystem* m_drivetrain = nullptr;
    IntakeSubsystem* m_intake = nullptr;

    units::angular_velocity::radians_per_second_t rotNote = units::angular_velocity::radians_per_second_t(0);
    double kpNote = 0.04;
    double txNote = 0.0;
    double tyNote = 0;

    bool NoInput = false;
    double noteError = 0;

    int state = 0;
    int time = 0;
    bool finished = false;

    int direction = 1;

    const units::velocity::meters_per_second_t driveSpeed = 2.5_mps; //the max possible speed
};
