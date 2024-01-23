// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterCmd.h"

ShooterCmd::ShooterCmd() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShooterCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShooterCmd::Execute() {}

// Called once the command ends or is interrupted.
void ShooterCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ShooterCmd::IsFinished() {
  return false;
}
