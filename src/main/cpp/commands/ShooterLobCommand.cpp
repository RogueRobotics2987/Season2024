// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterLobCommand.h"

ShooterLobCommand::ShooterLobCommand(){}
ShooterLobCommand::ShooterLobCommand(
  ShooterSubsystem &shooter,
  IntakeSubsystem &intake,
  frc::XboxController &driverController,
  ShooterWheelsSubsystem &shooterWheels,
  LightSubsystem &lights)
{
  m_lights = &lights;
  m_shooter = &shooter;
  m_intake = &intake;
  m_shooterWheels = &shooterWheels;
  m_driverController = &driverController;

  AddRequirements({m_lights});
  AddRequirements({m_shooter});
  AddRequirements({m_intake});
  AddRequirements({m_shooterWheels});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShooterLobCommand::Initialize() 
{
  m_shooterWheels->SetShooter(0.5, 0.5); //change speed
  hasShot = false; 
  time = 0;
}

// Called repeatedly when this Command is scheduled to run
void ShooterLobCommand::Execute() 
{
  m_shooter->SetActuator(45.5); //change angle

  if(m_driverController->GetRightTriggerAxis() > 0.05)
  {
    m_intake->RunMagazine(1);
    m_intake->RunIntake(1);
    m_intake->DirectionNote(1);

    hasShot = true; 
  }

  if(hasShot) 
  {
    time++;
  }
}


// Called once the command ends or is interrupted.
void ShooterLobCommand::End(bool interrupted) 
{
  m_shooterWheels->StopShooter();

  if(IsFinished() == true)
  {
    m_lights->SetNoColor();
  } 
  else 
  {  
    m_lights->SetLightsGreen();
  }
}

// Returns true when the command should end.
bool ShooterLobCommand::IsFinished() 
{
  if(time > 75) 
  {
    return true;
  }
  else 
  {
    return false;
  }
}
