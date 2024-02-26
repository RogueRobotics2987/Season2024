// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagFollower.h"

AprilTagFollower::AprilTagFollower(){}
AprilTagFollower::AprilTagFollower(LimelightSubsystem &limelight, DriveSubsystem &drivetrain, frc::XboxController &Xbox, ShooterSubsystem &shooter) 
{
  m_limelight = &limelight;
  m_drivetrain = &drivetrain;
  m_Xbox = &Xbox;
  m_shooter = &shooter;
  AddRequirements({m_limelight});
  AddRequirements({m_drivetrain});  
  AddRequirements({m_shooter});
}

// Called when the command is initially scheduled.
void AprilTagFollower::Initialize()
{
  // nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->PutNumber("pipeline",1);
}

// Called repeatedly when this Command is scheduled to run
void AprilTagFollower::Execute()
{
  double tx = m_limelight->FilteredPhotonYaw();//m_limelight->GetAprilTagtx() - 5;
  units::angular_velocity::radians_per_second_t rot = units::angular_velocity::radians_per_second_t(0);
  // if(tx > 7 || tx < -7){
  rot = units::angular_velocity::radians_per_second_t((0-tx) * kp);

  speedY = Deadzone(m_Xbox->GetLeftY());
  speedX = Deadzone(m_Xbox->GetLeftY());

  if((fabs(speedY) + fabs(speedX) + fabs(rot.value())) < .05)
  {
    NoJoystickInput = true;
  }
  else
  {
    NoJoystickInput = false;
  }
  // }
  // else{
  // rot = units::angular_velocity::radians_per_second_t(0);
  // }
  
  m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(speedX), rot, false, NoJoystickInput);
  //m_shooter->SetActuator(m_shooter->GetCurrentCommand);
}

// Called once the command ends or is interrupted.
void AprilTagFollower::End(bool interrupted) {}

// Returns true when the command should end.
bool AprilTagFollower::IsFinished()
{
  return false;
}

float AprilTagFollower::Deadzone(float x)
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