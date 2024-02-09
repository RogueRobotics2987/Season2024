// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/ArmSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/ColorSensorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/FunctionalCommand.h>

#include <frc/smartdashboard/SmartDashboard.h> 


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class StateMachine
    : public frc2::CommandHelper<frc2::Command, StateMachine> {
 public:
  StateMachine();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  enum intakeState {EMPTY, PICKUP, LOADED, SHOOTER_WARMUP, SHOOT, DROP_WARMUP, DROP};
  intakeState state = EMPTY;


  ArmSubsystem m_arm;
  ClimberSubsystem m_climb;
  ColorSensorSubsystem m_colorSensor;
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;

  int time = 0;
  int timeDrop = 0;
  bool orangeCheerio = false;     // if auto/teleop want to pickup a note

  bool warmMilk = false;          // warmup shooter
  bool spoon = false;             // activate shooter

  bool spillMilk = false;         // warmup dropper, move arm into position
  bool micdrop = false;           // activate dropper
  
};
