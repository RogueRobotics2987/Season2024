// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "networktables/NetworkTableInstance.inc"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LimelightPose.h"
/*
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoAprilTag
    : public frc2::CommandHelper<frc2::Command, AutoAprilTag>
{
  public:
    AutoAprilTag(); 
    AutoAprilTag(LimelightPose &limePose, DriveSubsystem &drivetrain);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    double kp = -0.09927;

  private:
    LimelightPose* m_limePose = nullptr;
    DriveSubsystem* m_drivetrain = nullptr;
    double tx = 0;
    double tv = 0;
};