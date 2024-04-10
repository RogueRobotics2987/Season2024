// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h> 

#include "ShootCommand.h" 
#include "RobotContainer.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ShooterWheelsSubsystem.h"
#include "subsystems/LightSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShooterLobCommand: public frc2::CommandHelper<frc2::Command, ShooterLobCommand> 
{
  public:
    ShooterLobCommand();
    ShooterLobCommand(
      ShooterSubsystem &shooter,
      IntakeSubsystem &intake,
      frc::XboxController &driverController,
      ShooterWheelsSubsystem &m_shooterWheels,
      LightSubsystem &lights);
    
    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

  private:
    LightSubsystem* m_lights = nullptr;
    ShooterSubsystem* m_shooter = nullptr;
    ShooterWheelsSubsystem* m_shooterWheels = nullptr;
    IntakeSubsystem* m_intake = nullptr;
    frc::XboxController* m_driverController = nullptr;
    double time = 0;
    bool hasShot = false; 
};
