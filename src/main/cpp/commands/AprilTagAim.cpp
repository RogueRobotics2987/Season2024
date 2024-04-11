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
  frc::XboxController &auxController,
  LightSubsystem &light) 
{
  m_limelight = &limelight;
  m_drivetrain = &drivetrain;
  m_driverController = &driveXbox;
  m_shooter = &shooter;
  m_auxController = &auxController;
  m_lights = &light;
  AddRequirements({m_limelight});
  AddRequirements({m_drivetrain});  
  AddRequirements({m_shooter});
  AddRequirements(m_lights);
}

// Called when the command is initially scheduled.
void AprilTagAim::Initialize()
{
  finished = false;
  shoot = false;
  time = 0;
  m_limelight->apriltagAngleReset(m_drivetrain->GetPose().Rotation().Degrees().value());
  currentHeading = m_drivetrain->GetPose().Rotation().Degrees().value();
  lastHeading = currentHeading;
  hasSeenTarget = false;

  m_driverController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);

}

// Called repeatedly when this Command is scheduled to run
void AprilTagAim::Execute()
{
  //calibrate these values with the at home field to make sure the deadbands are okay
  if(fabs(m_shooter->ShooterError()) <  1 && fabs(m_limelight->GetApriltagDriveError()) < 2 && m_limelight->GetNumTargets() > 0)
  {
    m_lights->SetFlashPurple();
    m_driverController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1);
  }
  else
  {
    m_lights->SetLightsPurple();
    m_driverController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
  }

  frc::SmartDashboard::PutBoolean("apriltagAim", true);

  m_shooter->AngleTrimAdjust(m_auxController->GetRawButtonPressed(6), m_auxController->GetRawButtonPressed(5));
  currentHeading = m_drivetrain->GetPose().Rotation().Degrees().value();

  rotApril = units::angular_velocity::radians_per_second_t(m_limelight->GetApriltagDriveMotorVal(currentHeading, lastHeading));

  frc::SmartDashboard::PutNumber("apriltagRotation", rotApril.value());
    
  m_shooter->SetActuator(m_limelight->GetApriltagShooterTheta(m_limelight->GetDistanceFromTarget(), m_shooter->GetAngleTrim()));

  speedY = Deadzone(m_driverController->GetLeftY());
  speedX = Deadzone(m_driverController->GetLeftX());
  rot = Deadzone(m_driverController->GetRightX());

  if((fabs(speedY) + fabs(speedX) + fabs(rot)) < .05)
  {
    NoJoystickInput = true;
  }
  else
  {
    NoJoystickInput = false;
  }
    
  if(hasSeenTarget == true)
  {
    if(fabs(rotApril.value()) > 0.05)
    {
      m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY * 4), units::velocity::meters_per_second_t(speedX * 4), rotApril, false, false);
    }
    else
    {
      m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY * 4), units::velocity::meters_per_second_t(speedX * 4), rotApril, false, NoJoystickInput);
    }
  }
  else
  {
    m_drivetrain->Drive(
      units::velocity::meters_per_second_t(speedY * 4),
      units::velocity::meters_per_second_t(speedX * 4),
      units::radians_per_second_t(-rot * AutoConstants::kMaxAngularSpeed),
      false,
      NoJoystickInput
    );
  }

  if(m_driverController->GetRightTriggerAxis() > 0.05)
  {
    shoot = true;
  }

  if(shoot == true)
  {
    time++;
  }

  if(time > 30)
  {
    finished = true;
  } 

  if(m_limelight->GetNumTargets() > 0)
  {
    hasSeenTarget = true;
  }

  //updating the last heading
  lastHeading = currentHeading;
}

// Called once the command ends or is interrupted.
void AprilTagAim::End(bool interrupted)
{
  frc::SmartDashboard::PutBoolean("apriltagAim", false);

  m_lights->SetNoColor();
  m_driverController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
}

// Returns true when the command should end.
bool AprilTagAim::IsFinished()
{
  return finished;
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