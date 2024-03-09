// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <iostream>
#include <frc/apriltag/AprilTagPoseEstimate.h>
//#include <frc/apriltag/AprilTagPoseEstimator.h>
#include "networktables/NetworkTableInstance.inc"

#include "Constants.h"
#include "photon/PhotonUtils.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"

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

    bool PhotonHasTarget();
    double PhotonYaw();
    double FilteredPhotonYaw();
    double PhotonYawMap(double originalVal);
    double FilteredDistance();
    photon::PhotonTrackedTarget GetFilteredTarget();
    double GetApriltagShooterTheta(double dist, double angleTrim);
    double DistanceBetweenAngles(double targetAngle, double sourceAngle);

    double GetApriltagDriveMotorVal(double currentHeading);
    double GetApriltagDriveError();

    double GetAmptx();

    std::vector<double> botPose;

 private:
  photon::PhotonCamera camera = photon::PhotonCamera("FrontCamera");
  photon::PhotonPipelineResult result;

  double AprilTagstx = 0;
  double AprilTagsty = 0;
  double Notetx = 0;
  double Notety = 0;

  bool hasTarget = false;

  std::vector<double> targetIDs;
  std::vector<photon::PhotonTrackedTarget> myTargets;
  std::span<const photon::PhotonTrackedTarget> tempTargets;
  double targetData = 0;
  photon::PhotonTrackedTarget filteredTarget;
  int filteredTargetID = -1;
  units::meter_t filteredRange = 0_m;

  units::meter_t CAMERA_HEIGHT = units::meter_t(0.635);
  units::meter_t TAREGT_HEIGHT = units::meter_t(1.5);
  units::angle::radian_t CAMERA_PITCH = units::angle::radian_t(0.45);

  units::meter_t AMP_TARGET_HEIGHT = units::meter_t(1.405);

  double driveError = 0;
  double txApril = 0;
  double desiredHeading = 0;
  double kpApril = 0.065;

    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};