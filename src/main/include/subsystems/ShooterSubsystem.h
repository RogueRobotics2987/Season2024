// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;



 private:
  rev::CANSparkMax* LeftShooter;
  rev::CANSparkMax* RightShooter;
  rev::CANSparkMax* ShooterActuator;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
