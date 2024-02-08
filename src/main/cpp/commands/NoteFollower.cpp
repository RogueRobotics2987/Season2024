// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NoteFollower.h"

NoteFollower::NoteFollower(){}
NoteFollower::NoteFollower(LimelightSubsystem &limePose, DriveSubsystem &drivetrain, frc::XboxController &Xbox)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_limePose = &limePose;
  m_drivetrain = &drivetrain;
  m_Xbox = &Xbox;
  AddRequirements({m_limePose});
  AddRequirements({m_drivetrain});
}

// Called when the command is initially scheduled.
void NoteFollower::Initialize()
{
  //  nt::NetworkTableInstance::GetDefault().GetTable("limelight-bac\k")->PutNumber("pipeline",0);
}

// Called repeatedly when this Command is scheduled to run
void NoteFollower::Execute() 
{
  double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx",0.0);
  if(tx > 7 || tx < -7){
    rot = units::angular_velocity::radians_per_second_t((0 - tx) * kp);
  }
  else
  {
    rot = units::angular_velocity::radians_per_second_t(0);
  }
  if(m_Xbox->GetLeftY() < 0.1 && m_Xbox->GetLeftY() > -0.1 && tx < 7 &&  tx > -7){
    speedY = 0; 
    NoJoystickInput = true;
  }
  else{
    speedY = m_Xbox->GetLeftY();
    NoJoystickInput = false;
  }

  m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(0), rot, false, NoJoystickInput);
}

// Called once the command ends or is interrupted.
void NoteFollower::End(bool interrupted) {}

// Returns true when the command should end.
bool NoteFollower::IsFinished()
{
  return false;
}
