// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>

#include "rev/CANSparkMax.h"
#include "DriveSubsystem.h"


class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();
  void Direction();
  void runIntake();
  void stopIntake();
  bool GetIntakeFront();
  bool GetIntakeRear();
 
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  

 private:
  double frontVal = 0.0;
  double backVal = 0.0;

  DriveSubsystem* m_drivetrain;

  rev::CANSparkMax BackIntake{9, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax FrontIntake{10, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax CenterIntake{11, rev::CANSparkMax::MotorType::kBrushless};

  frc::DigitalInput intakeColorSensorFront {1};   // 0 is a place holder for the DIO port
  frc::DigitalInput intakeColorSensorRear {2};   // 0 is a place holder for the DIO port


  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
