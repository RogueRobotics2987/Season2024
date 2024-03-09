// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootCommand.h"

ShootCommand::ShootCommand() {}

ShootCommand::ShootCommand(
  ShooterWheelsSubsystem &shooterWheels,
  IntakeSubsystem &intake,
  frc::XboxController &driverController,
  frc::XboxController &auxController) 
{
  m_shooterWheels = &shooterWheels;
  AddRequirements({m_shooterWheels});
  m_intake = &intake;
  m_driverController = &driverController;
  m_auxController = &auxController;
  AddRequirements({m_intake});
}

// Called when the command is initially scheduled.
void ShootCommand::Initialize() 
{
  hasShot = false;
  finished = false;
  time = 0;

  shooterWarmup();
}

// Called repeatedly when this Command is scheduled to run
void ShootCommand::Execute() 
{
  if(time >= 25 && hasShot == true)
  {
    finished = true;
  }
  else if(m_driverController->GetRightTriggerAxis() > 0.05)
  {
    shoot();
    time = 0;
    hasShot = true;
  }

  time++;
}

void ShootCommand::shooterWarmup()
{
  m_shooterWheels->PIDShoot();
  //m_shooter->SetShooter(0.75, 0.75);
}

void ShootCommand::shoot()
{
  m_intake->runIntake(1);
  m_intake->runMagazine(1);
  m_intake->DirectionNote(1);
}

void ShootCommand::stopShoot()
{
  m_intake->stopIntake();
  m_intake->stopMagazine();
  m_shooterWheels->StopShooter();
}

// Called once the command ends or is interrupted.
void ShootCommand::End(bool interrupted)
{
  m_shooterWheels->StopShooter();
}

// Returns true when the command should end.
bool ShootCommand::IsFinished() 
{
  return finished;
}
