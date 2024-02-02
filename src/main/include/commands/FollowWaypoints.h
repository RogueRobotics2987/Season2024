// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <math.h>
#include <list>
#include <frc/trajectory/TrapezoidProfile.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FollowWaypoints
    : public frc2::CommandHelper<frc2::Command, FollowWaypoints>
{
  public:
    FollowWaypoints();
    FollowWaypoints(DriveSubsystem &drivetrain, std::vector<frc::Pose2d> waypoints, units::meters_per_second_t driveSpeed);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    double DistanceBetweenAngles(double angle1, double angle2);


  private:
    DriveSubsystem* m_drivetrain = nullptr;
    std::list<frc::Pose2d> m_waypoints;

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
};
