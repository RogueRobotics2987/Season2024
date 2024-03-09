// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoAprilTag.h"

AutoAprilTag::AutoAprilTag() {}
AutoAprilTag::AutoAprilTag(LimelightSubsystem &limePose, DriveSubsystem &drivetrain, ShooterSubsystem &shooter)
{
  m_limePose = &limePose;
  m_drive = &drivetrain;
  m_shooter = &shooter;
  AddRequirements({m_limePose});
  AddRequirements({m_drive});
  AddRequirements({m_shooter});
}

// Called when the command is initially scheduled.
void AutoAprilTag::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoAprilTag::Execute()
{
  currentHeading = m_drive->GetPose().Rotation().Degrees().value();

  rotApril = units::angular_velocity::radians_per_second_t(m_limePose->GetApriltagDriveMotorVal(currentHeading, m_drive->GetPose().X().value(), m_drive->GetPose().Y().value()));
  
  if(m_limePose->FilteredDistance() == 0)
  {
    rotApril = 0_rad_per_s;
  }

  if(fabs(rotApril.value()) > 0.05)
  {
    m_drive->Drive(units::velocity::meters_per_second_t(0), units::velocity::meters_per_second_t(0), rotApril, false, false);
  }
  else
  {
    m_drive->Drive(units::velocity::meters_per_second_t(0), units::velocity::meters_per_second_t(0), rotApril, false, true);
  }

  m_shooter->SetActuator(m_limePose->GetApriltagShooterTheta(m_limePose->FilteredDistance(), m_shooter->GetAngleTrim()));
}

// Called once the command ends or is interrupted.
void AutoAprilTag::End(bool interrupted)
{
  m_drive->Drive(0_rad_per_s, false, false);
}

// Returns true when the command should end.
bool AutoAprilTag::IsFinished()
{
    return false;
}

double AutoAprilTag::DistanceBetweenAngles(double targetAngle, double sourceAngle)
{
  double a = targetAngle - sourceAngle;
  if(a > 180)
  {
    a = a + -360;
  }
  else if(a < -180)
  {
    a = a + 360;
  }
  else
  {
    a = a;
  }

  return a;
}
