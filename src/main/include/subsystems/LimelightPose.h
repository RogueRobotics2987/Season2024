// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <iostream>

#include "networktables/NetworkTableInstance.inc"

class LimelightPose : public frc2::SubsystemBase
{
  public:
    LimelightPose();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    std::vector<double> botPose;

 private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};