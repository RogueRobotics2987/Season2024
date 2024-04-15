// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LightSubsystem.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.inc"
#include <frc/smartdashboard/SmartDashboard.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IntakeCmd : public frc2::CommandHelper<frc2::Command, IntakeCmd>
{
  public:
    IntakeCmd();
    IntakeCmd(IntakeSubsystem &intake, LightSubsystem &light, frc::XboxController &driverController, frc::XboxController &auxController);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

  private:
    IntakeSubsystem* m_intake = nullptr;
    LightSubsystem* m_lights = nullptr;
    frc::XboxController* m_driverController = nullptr;
    frc::XboxController* m_auxController = nullptr;

    int state = 0;
    int time = 0;
    bool finished = false;
};
