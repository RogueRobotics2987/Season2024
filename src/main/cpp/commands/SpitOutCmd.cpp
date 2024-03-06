// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SpitOutCmd.h"

SpitOutCmd::SpitOutCmd(IntakeSubsystem &intake)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_intake = &intake;
  AddRequirements({m_intake});
}

// Called when the command is initially scheduled.
void SpitOutCmd::Initialize()
{

}

// Called repeatedly when this Command is scheduled to run
void SpitOutCmd::Execute()
{
  m_intake->spitOutIntake();
}

// Called once the command ends or is interrupted.
void SpitOutCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool SpitOutCmd::IsFinished() {
  return false;
}
