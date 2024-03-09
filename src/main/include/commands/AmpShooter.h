// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include "RobotContainer.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ShooterWheelsSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AmpShooter : public frc2::CommandHelper<frc2::Command, AmpShooter>
{
  public:
    AmpShooter();
    AmpShooter(
      ShooterSubsystem &shooter,
      IntakeSubsystem &intake,
      frc::XboxController &driverController,
      ArmSubsystem &arm,
      ShooterWheelsSubsystem &m_shooterWheels);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

  private:
    ShooterSubsystem* m_shooter = nullptr;
    ShooterWheelsSubsystem* m_shooterWheels = nullptr;
    IntakeSubsystem* m_intake = nullptr;
    frc::XboxController* m_driverController = nullptr;
    ArmSubsystem* m_arm = nullptr;
    double time = 0;
    bool hasShot = false; 
};

