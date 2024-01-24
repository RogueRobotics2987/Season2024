// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class followWaypoints
    : public frc2::CommandHelper<frc2::Command, followWaypoints> {
  public:
    followWaypoints();
    followWaypoints(DriveSubsystem &drivetrain, std::vector<frc::Pose2d> waypoints, units::meter_t driveSpeed);


    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    bool finished;
    frc::Pose2d currentPose;
    frc::Pose2d desiredPose;
  private:
    DriveSubsystem* m_drivetrain = nullptr;
    std::vector<frc::Pose2d> m_waypoints;

};
