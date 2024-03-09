// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ShooterWheelsSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootCommand
    : public frc2::CommandHelper<frc2::Command, ShootCommand>
{
 public:
  ShootCommand();
  ShootCommand(ShooterWheelsSubsystem &shooterWheels,
                IntakeSubsystem &intake,
                frc::XboxController &driverController,
                frc::XboxController &auxController
  );

  void shooterWarmup();
  void shoot();
  void stopShoot();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterWheelsSubsystem* m_shooterWheels = nullptr;
  IntakeSubsystem* m_intake = nullptr;
  frc::XboxController* m_driverController = nullptr;
  frc::XboxController* m_auxController = nullptr;

  double time = 0;
  bool finished = false;
  bool hasShot = false;

};
