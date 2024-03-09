// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoShootCommand.h"

AutoShootCommand::AutoShootCommand() {}

AutoShootCommand::AutoShootCommand(ShooterWheelsSubsystem &shooterWheel,
                            IntakeSubsystem &intake) 
{
  m_shooterWheel = &shooterWheel;
  AddRequirements({m_shooterWheel});
  m_intake = &intake;
  AddRequirements({m_intake});
}

// Called when the command is initially scheduled.
void AutoShootCommand::Initialize() 
{  
  finished = false;
  time = 0;
}

// Called repeatedly when this Command is scheduled to run
void AutoShootCommand::Execute() 
{
  if(time <= 60) //todo change values to be most efficient/effective
  {
    ShooterWarmup();
  }
  else if(60 <= time && time <= 100)
  {
    shoot();
  }
  else if(time >= 100)
  {
    stopShoot();
    finished = true;
  }

  time++;
}

void AutoShootCommand::ShooterWarmup()
{
  m_shooterWheel->SetShooter(0.75, 0.75);
}

void AutoShootCommand::shoot()
{
  m_intake->RunIntake(1);
  m_intake->RunMagazine(1);
  m_intake->DirectionNote(1);
}

void AutoShootCommand::stopShoot()
{
  m_intake->StopIntake();
  m_intake->StopMagazine();
  m_shooterWheel->StopShooter();
}

// Called once the command ends or is interrupted.
void AutoShootCommand::End(bool interrupted){}

// Returns true when the command should end.
bool AutoShootCommand::IsFinished() 
{
  return finished;
}
