// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "rev/AbsoluteEncoder.h"


class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  void setLowerArmAngle(double desiredAngle);
  void setUpperArmAngle(double desiredAngle);

  void defaultArmPos();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::CANSparkMax LowerArm{17, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax UpperArm{18, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax ArmWheels{19, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxAlternateEncoder* m_LowerArmEncoder = new rev::SparkMaxAlternateEncoder(LowerArm.GetAlternateEncoder(125));
  rev::SparkMaxAlternateEncoder* m_UpperArmEncoder = new rev::SparkMaxAlternateEncoder(UpperArm.GetAlternateEncoder(34.375));  //placeholder, TODO: test


  double kpLowerArm = 0.0111;  //TODO: test
  double kpUpperArm = 0.00555;

  double kiLowerArm = 0.000111;  //TODO: test
  double kiUpperArm = 0.0000555;

  double kiSumLowerArm = 0.0;
  double kiSumUpperArm = 0.0;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
