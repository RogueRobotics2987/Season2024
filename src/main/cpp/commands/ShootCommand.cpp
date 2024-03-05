// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootCommand.h"

ShootCommand::ShootCommand()
{

}

ShootCommand::ShootCommand(ShooterSubsystem &shooter,
                            IntakeSubsystem &intake) 
{
  m_shooter = &shooter;
  AddRequirements({m_shooter});
  m_intake = &intake;
  AddRequirements({m_intake});
}

// Called when the command is initially scheduled.
void ShootCommand::Initialize() 
{

}

// Called repeatedly when this Command is scheduled to run
void ShootCommand::Execute() 
{

}

void ShootCommand::shooterWarmup()
{

}

void ShootCommand::shoot()
{
  
}

// Called once the command ends or is interrupted.
void ShootCommand::End(bool interrupted) 
{

}

// Returns true when the command should end.
bool ShootCommand::IsFinished() 
{
  return false;
}
