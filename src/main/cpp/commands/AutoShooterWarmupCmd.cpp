// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoShooterWarmupCmd.h"

AutoShooterWarmupCmd::AutoShooterWarmupCmd(ShooterWheelsSubsystem &shooterWheel)
{
  m_shooterWheel = &shooterWheel;
  AddRequirements({m_shooterWheel});
}

// Called when the command is initially scheduled.
void AutoShooterWarmupCmd::Initialize()
{
  m_shooterWheel->PIDShoot();
}

// Called repeatedly when this Command is scheduled to run
void AutoShooterWarmupCmd::Execute()
{
  //do nothing
}

// Called once the command ends or is interrupted.
void AutoShooterWarmupCmd::End(bool interrupted){}

// Returns true when the command should end.
bool AutoShooterWarmupCmd::IsFinished() {
  return true;
}
