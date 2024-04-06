// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoShootCommand.h"

AutoShootCommand::AutoShootCommand() {}

AutoShootCommand::AutoShootCommand(ShooterWheelsSubsystem &shooterWheel,
                                    IntakeSubsystem &intake,
                                    ShooterSubsystem &shooter) 
{
  m_shooterWheel = &shooterWheel;
  AddRequirements({m_shooterWheel});
  m_intake = &intake;
  AddRequirements({m_intake});
  m_shooter = &shooter;
  AddRequirements({m_shooter});
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

  if(time <= 25)
  {
    shoot();
  }
  else if(time >= 25)
  {
    stopShoot();
    finished = true;
  }

  time++;
}

void AutoShootCommand::ShooterWarmup()
{
  m_shooterWheel->PIDShoot();
}

void AutoShootCommand::shoot()
{
  m_intake->RunIntake(1);
  m_intake->RunMagazine(1);
  m_intake->DirectionNote(1); // why run this?
}

void AutoShootCommand::stopShoot()
{
  m_intake->StopIntake();
  m_intake->StopMagazine();
}

// Called once the command ends or is interrupted.
void AutoShootCommand::End(bool interrupted)
{
  m_shooter->setRestingActuatorPosition();
}

// Returns true when the command should end.
bool AutoShootCommand::IsFinished() 
{
  return finished;
}
