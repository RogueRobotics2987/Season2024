// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <iostream>

#include <frc/controller/PIDController.h>
#include <units/angle.h>
#include <units/length.h>

#include "networktables/NetworkTableInstance.inc"

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

    std::vector<double> botPose;

 private:
  photon::PhotonCamera camera = photon::PhotonCamera("FrontCamera");
  photon::PhotonPipelineResult result = camera.GetLatestResult();

  double AprilTagstx = 0;
  double AprilTagsty = 0;
  double Notetx = 0;
  double Notety = 0;
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};