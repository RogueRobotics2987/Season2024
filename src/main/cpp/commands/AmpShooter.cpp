// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpShooter.h"

AmpShooter::AmpShooter() {}
AmpShooter::AmpShooter(
  ShooterSubsystem &shooter, 
  IntakeSubsystem &intake,
  frc::XboxController &driverController,
  ArmSubsystem &arm,
  ShooterWheelsSubsystem &shooterWheels,
  LightSubsystem &lights) 
{
  m_shooter = &shooter;
  m_intake = &intake;
  m_arm = &arm;
  m_shooterWheels = &shooterWheels;
  m_driverController = &driverController;
  m_lights = &lights;
  AddRequirements({m_lights});
  AddRequirements({m_shooter});
  AddRequirements({m_shooterWheels});
  AddRequirements({m_intake});
  AddRequirements({m_arm});
}

// Called when the command is initially scheduled.
void AmpShooter::Initialize() 
{
  m_shooterWheels->SetShooter(0.4, 0.4);
  hasShot = false; 
  time = 0;
}

// Called repeatedly when this Command is scheduled to run
void AmpShooter::Execute() 
{
  m_shooter->SetActuator(45.5);

  if(m_arm->GetOffSetEncoderValueLower() > 50)
  {
    m_arm->RunArmWheels(-1);
  }

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
void AmpShooter::End(bool interrupted) 
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
bool AmpShooter::IsFinished() 
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
