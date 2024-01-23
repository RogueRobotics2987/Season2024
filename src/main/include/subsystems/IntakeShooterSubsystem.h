// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h> 
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/motorcontrol/PWMSparkMax.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  enum intakeState {EMPTY, PICKUP, MAGAZINE, LOADED, SHOOTER_WARMUP, SHOOT};
  intakeState state = EMPTY;


  rev::CANSparkMax* m_intakeMotor1 = new rev::CANSparkMax(1 /*m_intakeMotor1*/, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_intakeMotor2 = new rev::CANSparkMax(2, rev::CANSparkMax::MotorType::kBrushless);
  // intakeMotore3 == middle motor that picks up note
  rev::CANSparkMax* m_intakeMotor3 = new rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless);

  rev::CANSparkMax* m_MagMotor1 = new rev::CANSparkMax(4, rev::CANSparkMax::MotorType::kBrushless);

  rev::CANSparkMax* m_shooterMotor1 = new rev::CANSparkMax(5, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_shooterMotor2 = new rev::CANSparkMax(6, rev::CANSparkMax::MotorType::kBrushless);

  /*int IntakeMotor1Port = 1;
  int IntakeMotor2Port = 2;
  int IntakeMotor3Port = 3; // middle motor that picks up piece
  int MagMotor1Port = 4;
  int shooterMotor1Port = 5;
  int shooterMotor2Port = 6;*/

  // intake sensor
  // magazine sensor

  bool orangeCheerio = false; // if a note is detected
  bool warmMilk = false;      // warm up shooter
  bool spoon = false;         // activate shooter

};
