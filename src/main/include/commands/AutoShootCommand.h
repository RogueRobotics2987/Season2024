// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ShooterWheelsSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoShootCommand
    : public frc2::CommandHelper<frc2::Command, AutoShootCommand>
{
  public:
    AutoShootCommand();
    AutoShootCommand(ShooterWheelsSubsystem &shooterWheels,
                  IntakeSubsystem &intake
    );

    void shooterWarmup();
    void shoot();
    void stopShoot();

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

  private:
    ShooterWheelsSubsystem* m_shooterWheel = nullptr;
    IntakeSubsystem* m_intake = nullptr;

    double time = 0;
    bool finished = false;
};
