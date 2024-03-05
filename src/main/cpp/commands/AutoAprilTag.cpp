// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoAprilTag.h"

AutoAprilTag::AutoAprilTag(){}
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
void AutoAprilTag::Initialize() 
{
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->PutNumber("pipeline",1);
}

// Called repeatedly when this Command is scheduled to run
void AutoAprilTag::Execute()
{
    if(m_limePose->PhotonHasTarget() == true)  //limited drive, else regular
    {
      filteredTargetID = m_limePose->GetFilteredTarget().GetFiducialId();

      currentHeading = m_drive->GetPose().Rotation().Degrees().value();

      if (filteredTargetID == 4 || filteredTargetID == 7)
      {
        txApril = m_limePose->FilteredPhotonYaw(); //m_limelight->GetAprilTagtx() - 5; // TODO: check
        desiredHeading = currentHeading + txApril;
      }

      frc::SmartDashboard::PutNumber("filtered yaw val", txApril);

      error = DistanceBetweenAngles(desiredHeading, currentHeading);

      rotApril = units::angular_velocity::radians_per_second_t(error * kpApril);
        
      m_drive->Drive(units::velocity::meters_per_second_t(0), units::velocity::meters_per_second_t(0), -rotApril, false, false);
    }
}

// Called once the command ends or is interrupted.
void AutoAprilTag::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAprilTag::IsFinished()
{
  if(error < 3)
  {
    return true;
  }
  else
  {
    return false;
  }
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
