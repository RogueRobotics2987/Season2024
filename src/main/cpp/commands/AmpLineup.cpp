// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpLineup.h"

AmpLineup::AmpLineup(DriveSubsystem &drive, LimelightSubsystem &LimeLight) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_drive = &drive;
  m_LimeLight = &LimeLight;
  AddRequirements(m_drive);
  AddRequirements(m_LimeLight);
}

// Called when the command is initially scheduled.
void AmpLineup::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AmpLineup::Execute() 
{
  // double Apriltx = m_LimeLight->GetAmptx();
  // m_drive->Drive(units::velocity::meters_per_second_t(-Apriltx* 0.1), units::velocity::meters_per_second_t(0), false, false);
}

// Called once the command ends or is interrupted.
void AmpLineup::End(bool interrupted) {}

// Returns true when the command should end.
bool AmpLineup::IsFinished() 
{
  return false;
}
