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
  ShooterWheelsSubsystem &shooterWheels)
{
  m_shooter = &shooter;
  m_intake = &intake;
  m_arm = &arm;
  m_shooterWheels = &shooterWheels;
  m_driverController = &driverController;
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
  m_shooter->SetActuator(37.5);
  m_arm->setLowerArmAngle(112);

  if(m_driverController->GetRightTriggerAxis() > 0.05)
  {
    m_intake->runMagazine(1);
    m_intake->runIntake(1);
    m_intake->DirectionNote(1);
    m_arm->runArmWheels(-1);

    hasShot = true; 
  }

    if(hasShot) {
      time += 1;
    }
}

// Called once the command ends or is interrupted.
void AmpShooter::End(bool interrupted) 
{
 m_shooterWheels->StopShooter();
}

// Returns true when the command should end.
bool AmpShooter::IsFinished() 
{
   if(time > 30) {
    return true;
  }
  else {
    return false;
  }
}
