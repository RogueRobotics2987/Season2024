// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"
#include "networktables/NetworkTableInstance.inc"
#include "subsystems/LimelightSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AmpLineup : public frc2::CommandHelper<frc2::Command, AmpLineup> 
{
  public:
    AmpLineup(
      DriveSubsystem &drive,
      LimelightSubsystem &LimeLight);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    double Apriltx = 0;

  private:
    DriveSubsystem* m_drive;
    LimelightSubsystem* m_LimeLight;
};
