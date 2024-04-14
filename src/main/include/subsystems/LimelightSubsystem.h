// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <iostream>
#include <frc/apriltag/AprilTagPoseEstimate.h>

#include "networktables/NetworkTableInstance.inc"
#include "Constants.h"

class LimelightSubsystem : public frc2::SubsystemBase
{
  public:
    LimelightSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    double GetAprilTagtx();
    double GetAprilTagty();
    double GetNotetx();
    double GetNotety();

    double GetApriltagShooterTheta(double dist, double angleTrim);
    double DistanceBetweenAngles(double targetAngle, double sourceAngle);
    void apriltagAngleReset(double currentHeading);

    double GetApriltagDriveMotorVal(double currentHeading, double lastHeading);
    double GetApriltagDriveError();
    double GetDistanceFromTarget();
    int GetNumTargets();

    void SetNoteSeen();
    bool GetHasSeenNote();

    double GetAmptx();

  private:
    double AprilTagstx = 0;
    double AprilTagsty = 0;
    double Notetx = 0;
    double Notety = 0;

    bool hasTarget = false;

    std::vector<double> bluePose;

    double driveError = 0;
    double txApril = 0;
    double desiredHeading = 0;
    double kpApril = 0.055;

    double targetCount = -1;

    double blueS = 5.45;

    double rz = 0; // current heading but from limelight
    double lastDistance= 0;
    double currentDistance = 0;

    bool hasSeenNote = false;
};