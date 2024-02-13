// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc/Servo.h>
//#include "networktables/NetworkTableInstance.inc"
#include <frc/smartdashboard/SmartDashboard.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void startClimber();
  void stopClimber();

 private:
  rev::CANSparkMax ClimberMoter{12, rev::CANSparkMax::MotorType::kBrushless};
  
};
