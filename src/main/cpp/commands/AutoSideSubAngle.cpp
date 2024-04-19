// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoSideSubAngle.h"

AutoSideSubAngle::AutoSideSubAngle(ShooterSubsystem &shooter)
{
  m_shooter = &shooter;
  AddRequirements({m_shooter});
}

// Called when the command is initially scheduled.
void AutoSideSubAngle::Initialize()
{
  m_shooter->SetActuator(ShooterConstants::SubwooferSideAngle); 
}

// Called repeatedly when this Command is scheduled to run
void AutoSideSubAngle::Execute()
{
  if(time >= 60)
  {
    timeIsUp = true;
  }
  else
  {
    time++;
  }
}

// Called once the command ends or is interrupted.
void AutoSideSubAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoSideSubAngle::IsFinished()
{
  return timeIsUp;
}
