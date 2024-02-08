// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void StopShooter();
  void SetShooter();
  void ReverseShooter();
  void SetActuator(double CurrentAngle, double DesiredAngle, double kp);


 private:
  rev::CANSparkMax LeftShooter{15, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax RightShooter{16, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax ShooterActuator{13, rev::CANSparkMax::MotorType::kBrushless};
  //Current value encoder value, desired value is an equation using Limelight. Set to 10 for now | (curAngle - desAngle) * kp = motorOutput | kp can start at 1/90, wil check with encode when WE ACTUALLY GET THE GOSH DIDILY DARN ROBOT
  // ctre::phoenix::sensors::CANCoder ShooterEncoder;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
