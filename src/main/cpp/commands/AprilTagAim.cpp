// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagAim.h"

AprilTagAim::AprilTagAim(){}
AprilTagAim::AprilTagAim(LimelightSubsystem &limelight, DriveSubsystem &drivetrain, frc::XboxController &driveXbox, ShooterSubsystem &shooter) 
{
  m_limelight = &limelight;
  m_drivetrain = &drivetrain;
  m_driverController = &driveXbox;
  m_shooter = &shooter;
  AddRequirements({m_limelight});
  AddRequirements({m_drivetrain});  
  AddRequirements({m_shooter});
}

// Called when the command is initially scheduled.
void AprilTagAim::Initialize(){}

// Called repeatedly when this Command is scheduled to run
void AprilTagAim::Execute()
{
  currentHeading = m_drivetrain->GetPose().Rotation().Degrees().value();

  rotApril = units::angular_velocity::radians_per_second_t(m_limelight->GetApriltagDriveMotorVal(currentHeading));
    
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
  
  m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(speedX), rotApril, false, NoJoystickInput);
}

// Called once the command ends or is interrupted.
void AprilTagAim::End(bool interrupted) {}

// Returns true when the command should end.
bool AprilTagAim::IsFinished()
{
  return false;
}

float AprilTagAim::Deadzone(float x)
{
  if ((x < 0.1) &&  (x > -0.1)){
    x=0;
  }
  else if (x >= 0.1){
    x = x - 0.1;
  }
  else if (x <= -0.1){
    x = x + 0.1;
  }
  return(x);
}