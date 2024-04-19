// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ShooterSubsystem.h"

class AutoSideSubAngle
    : public frc2::CommandHelper<frc2::Command, AutoSideSubAngle>
{
  public:
    AutoSideSubAngle();
    AutoSideSubAngle(ShooterSubsystem &shooter);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override; 

    bool IsFinished() override;

    private:
      ShooterSubsystem* m_shooter = nullptr;

      int time = 0;
      bool timeIsUp = false;
};
