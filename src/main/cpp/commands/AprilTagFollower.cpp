// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagFollower.h"

AprilTagFollower::AprilTagFollower(){}
AprilTagFollower::AprilTagFollower(LimelightSubsystem &limePose, DriveSubsystem &drivetrain, frc::XboxController &Xbox) 
{
  m_limePose = &limePose;
  m_drivetrain = &drivetrain;
  m_Xbox = &Xbox;
  AddRequirements({m_limePose});
  AddRequirements({m_drivetrain});
}

// Called when the command is initially scheduled.
void AprilTagFollower::Initialize()
{
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->PutNumber("pipeline",1);
}

// Called repeatedly when this Command is scheduled to run
void AprilTagFollower::Execute()
{
  double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tx",0.0);
  units::angular_velocity::radians_per_second_t rot = units::angular_velocity::radians_per_second_t(0);
  // if(tx > 7 || tx < -7){
  rot = units::angular_velocity::radians_per_second_t((0-tx) * kp);
  // }
  // else{
  // rot = units::angular_velocity::radians_per_second_t(0);
  // }
  m_drivetrain->Drive(units::velocity::meters_per_second_t(m_Xbox->GetLeftY()), units::velocity::meters_per_second_t(m_Xbox->GetLeftX()), rot, false, false);
}

// Called once the command ends or is interrupted.
void AprilTagFollower::End(bool interrupted) {}

// Returns true when the command should end.
bool AprilTagFollower::IsFinished()
{
  return false;
}