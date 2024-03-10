// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpLineup.h"

AmpLineup::AmpLineup(
  DriveSubsystem &drive,
  LimelightSubsystem &LimeLight,
  frc::XboxController &driveXbox) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_drive = &drive;
  m_LimeLight = &LimeLight;
  m_driverController = &driveXbox;
  AddRequirements(m_drive);
  AddRequirements(m_LimeLight);
}

// Called when the command is initially scheduled.
void AmpLineup::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AmpLineup::Execute() 
{
  ampCurrentHeading = m_drive->GetPose().Rotation().Degrees().value();

  ampRotApril = units::angular_velocity::radians_per_second_t(m_LimeLight->GetAmpApriltagDriveMotorVal(ampCurrentHeading));

  ampSpeedY = Deadzone(m_driverController->GetLeftY());
  ampSpeedX = Deadzone(m_driverController->GetLeftX());

  if((fabs(ampSpeedY) + fabs(ampSpeedX)) < .05)
  {
    ampNoJoystickInput = true;
  }
  else
  {
    ampNoJoystickInput = false;
  }

  if(m_LimeLight->FilteredDistance() == 0)
  {
    ampRotApril = 0_rad_per_s;
  }
  
  if(fabs(ampRotApril.value()) > 0.05)
  {
    m_drive->Drive(units::velocity::meters_per_second_t(-ampSpeedY * 4), units::velocity::meters_per_second_t(-ampSpeedX * 4), ampRotApril, true, false);
  }
  else
  {
    m_drive->Drive(units::velocity::meters_per_second_t(-ampSpeedY * 4), units::velocity::meters_per_second_t(-ampSpeedX * 4), ampRotApril, true, ampNoJoystickInput);
  }
}

// Called once the command ends or is interrupted.
void AmpLineup::End(bool interrupted) {}

// Returns true when the command should end.
bool AmpLineup::IsFinished() 
{
  return false;
}

float AmpLineup::Deadzone(float x)
{
  if ((x < 0.1) && (x > -0.1))
  {
    x = 0;
  }
  else if (x >= 0.1)
  {
    x = x - 0.1;
  }
  else if (x <= -0.1)
  {
    x = x + 0.1;
  }
  return(x);
}
