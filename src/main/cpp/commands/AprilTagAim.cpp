// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagAim.h"

AprilTagAim::AprilTagAim() {}
AprilTagAim::AprilTagAim(
  LimelightSubsystem &limelight,
  DriveSubsystem &drivetrain,
  frc::XboxController &driveXbox,
  ShooterSubsystem &shooter,
  frc::XboxController &auxController) 
{
  m_limelight = &limelight;
  m_drivetrain = &drivetrain;
  m_driverController = &driveXbox;
  m_shooter = &shooter;
  m_auxController = &auxController;
  AddRequirements({m_limelight});
  AddRequirements({m_drivetrain});  
  AddRequirements({m_shooter});
}

// Called when the command is initially scheduled.
void AprilTagAim::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AprilTagAim::Execute()
{
  frc::SmartDashboard::PutBoolean("apriltagAim", true);

  m_shooter->AngleTrimAdjust(m_auxController->GetRawButtonPressed(6), m_auxController->GetRawButtonPressed(5));
  currentHeading = m_drivetrain->GetPose().Rotation().Degrees().value();

  rotApril = units::angular_velocity::radians_per_second_t(m_limelight->GetApriltagDriveMotorVal(currentHeading));

  frc::SmartDashboard::PutNumber("apriltagRotation", rotApril.value());
    
  m_shooter->SetActuator(m_limelight->GetApriltagShooterTheta(m_limelight->FilteredDistance(), m_shooter->GetAngleTrim()));

  speedY = Deadzone(m_driverController->GetLeftY());
  speedX = Deadzone(m_driverController->GetLeftY());

  if((fabs(speedY) + fabs(speedX)) < .05)
  {
    NoJoystickInput = true;
  }
  else
  {
    NoJoystickInput = false;
  }

  if(m_limelight->FilteredDistance() == 0)
  {
    rotApril = 0_rad_per_s;
  }
  
  if(fabs(rotApril.value()) > 0.05)
  {
    m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(speedX), rotApril, false, false);
  }
  else
  {
    m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(speedX), rotApril, false, NoJoystickInput);
  }
}

// Called once the command ends or is interrupted.
void AprilTagAim::End(bool interrupted)
{
  frc::SmartDashboard::PutBoolean("apriltagAim", false);
}

// Returns true when the command should end.
bool AprilTagAim::IsFinished()
{
  return false;
}

float AprilTagAim::Deadzone(float x)
{
  if ((x < 0.1) &&  (x > -0.1))
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