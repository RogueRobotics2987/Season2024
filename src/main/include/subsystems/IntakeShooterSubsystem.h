// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"


#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h> 
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <hal/can.h> 

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  frc2::CommandPtr Pickup();
  frc2::CommandPtr PickupStop();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  enum intakeState {EMPTY, PICKUP, /*MAGAZINE,*/ LOADED, SHOOTER_WARMUP, SHOOT, DROP_WARMUP, DROP};
  intakeState state = EMPTY;


  rev::CANSparkMax* m_intakeMotor1 = new rev::CANSparkMax(1 /*m_intakeMotor1*/, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_intakeMotor2 = new rev::CANSparkMax(2, rev::CANSparkMax::MotorType::kBrushless);
  // intakeMotore3 == middle motor that picks up note
  rev::CANSparkMax* m_intakeMotor3 = new rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless);

  rev::CANSparkMax* m_magMotor1 = new rev::CANSparkMax(4, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_magMotor2 = new rev::CANSparkMax(7, rev::CANSparkMax::MotorType::kBrushless); // mag motors on the hood

  rev::CANSparkMax* m_shooterMotor1 = new rev::CANSparkMax(5, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_shooterMotor2 = new rev::CANSparkMax(6, rev::CANSparkMax::MotorType::kBrushless);

  frc::DigitalInput colorInput {0};   // 0 is a place holder for the DIO port
  frc::DigitalInput colorInput2 {1};  // 1 is a place holder for the DIO port
  

  bool orangeCheerio = false;     // if auto/teleop want to pickup a note

  bool detectiveOrange1 = false;  // color sensor on the __ of robot
  //bool detectiveOrange2 = false;  // color sensor on the __ of robot
  bool eatenCheerio = false;      // color sensor on hood

  bool warmMilk = false;          // warmup shooter
  bool spoon = false;             // activate shooter

  bool spillMilk = false;         // warmup dropper, move arm into position
  bool micdrop = false;           // activate dropper
  
};
