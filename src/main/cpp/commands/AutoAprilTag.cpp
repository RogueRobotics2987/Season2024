// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoAprilTag.h"

AutoAprilTag::AutoAprilTag(){}
AutoAprilTag::AutoAprilTag(LimelightSubsystem &limePose, DriveSubsystem &drivetrain)
{
  m_limePose = &limePose;
  m_drivetrain = &drivetrain;
  AddRequirements({m_limePose});
  AddRequirements({m_drivetrain});
}

// Called when the command is initially scheduled.
void AutoAprilTag::Initialize() 
{
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->PutNumber("pipeline",1);
}

// Called repeatedly when this Command is scheduled to run
void AutoAprilTag::Execute()
{
  tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tx",0);
  tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tv",0);
  units::angular_velocity::radians_per_second_t rot = units::angular_velocity::radians_per_second_t(0);

  if( tx != -9999)
  {
    rot = units::angular_velocity::radians_per_second_t((0-tx) * kp);
  }
  else
  {
    rot = units::angular_velocity::radians_per_second_t(0);
  }
  
  m_drivetrain->Drive(units::velocity::meters_per_second_t(0), units::velocity::meters_per_second_t(0), rot, false, false);
}

// Called once the command ends or is interrupted.
void AutoAprilTag::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAprilTag::IsFinished()
{
  if(tv > 0 && fabs(tx)< 0.5)
  {
    return true;
  }
  else
  {
    return false;
  }
}