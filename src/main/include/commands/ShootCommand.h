// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootCommand
    : public frc2::CommandHelper<frc2::Command, ShootCommand> {
 public:
  ShootCommand();
  ShootCommand(ShooterSubsystem &shooter,
                IntakeSubsystem &intake);

  void shooterWarmup();
  void shoot();
  void stopShoot(); //was deleted in ShootCommand.cpp

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_shooter = nullptr;
  IntakeSubsystem* m_intake = nullptr;

};
