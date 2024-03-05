// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCmd.h"

IntakeCmd::IntakeCmd(IntakeSubsystem &intake)
{
  m_intake = &intake;
  AddRequirements(m_intake);
}

// Called when the command is initially scheduled.
void IntakeCmd::Initialize()
{
  m_intake->runIntake(0.25);
  m_intake->Direction(0.25);
  // m_intake->run(0.25); arm wheels need to move too but what build is doing with it is unknown
  m_intake->runMagazine(0.25);
}

// Called repeatedly when this Command is scheduled to run
void IntakeCmd::Execute()
{
  //doesnt need to do anything repeatedly
}

// Called once the command ends or is interrupted.
void IntakeCmd::End(bool interrupted)
{
  m_intake->runIntake(0);
  m_intake->Direction(0);
  // m_intake->run(0); arm wheels need to move too but what build is doing with it is unknown
  m_intake->runMagazine(0);
}

// Returns true when the command should end.
bool IntakeCmd::IsFinished()
{
  m_intake->GetMagazineSensor();
}
