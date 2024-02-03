// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <iostream>

#include "networktables/NetworkTableInstance.inc"

class LimelightSubsystem : public frc2::SubsystemBase
{
  public:
    LimelightSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    double GetAprilTagtx();
    double GetNotetx();

    std::vector<double> botPose;

    double AprilTagstx = 0;
    double Notetx = 0;

 private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};