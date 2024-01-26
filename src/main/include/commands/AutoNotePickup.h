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
#include "subsystems/LimelightPose.h"
 
class AutoNotePickup
    : public frc2::CommandHelper<frc2::Command, AutoNotePickup>
{
  public:
    AutoNotePickup();
    AutoNotePickup(LimelightPose &limePose, DriveSubsystem &drivetrain, frc::XboxController &Xbox);


    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    units::angular_velocity::radians_per_second_t rot = units::angular_velocity::radians_per_second_t(0);
    units::velocity::meters_per_second_t speed = units::velocity::meters_per_second_t(0);
    double kp = 0.09927;

  private:
    LimelightPose* m_limePose = nullptr;
    DriveSubsystem* m_drivetrain = nullptr;
    frc::XboxController* m_Xbox = nullptr;
};
