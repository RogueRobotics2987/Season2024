// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "networktables/NetworkTableInstance.inc"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LimelightSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
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
    AutoAprilTag(LimelightSubsystem &limePose, DriveSubsystem &drivetrain, ShooterSubsystem &m_shooter);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    double DistanceBetweenAngles(double targetAngle, double sourceAngle);

  private:
    LimelightSubsystem* m_limePose = nullptr;
    DriveSubsystem* m_drive = nullptr;
    ShooterSubsystem* m_shooter = nullptr;

    units::angular_velocity::radians_per_second_t rotApril = units::angular_velocity::radians_per_second_t(0);
    double kpApril = 0.09;
    double txApril = 0.0;

    double currentHeading = 0;
    double desiredHeading = 0;
    int filteredTargetID = -1;
    double error = 0;

};