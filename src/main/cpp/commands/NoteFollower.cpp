// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NoteFollower.h"

NoteFollower::NoteFollower(){}
NoteFollower::NoteFollower(LimelightSubsystem &limelight, DriveSubsystem &drivetrain, frc::XboxController &Xbox, IntakeSubsystem &intake, ShooterSubsystem &shooter, ArmSubsystem & arm)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_limelight = &limelight;
  m_drivetrain = &drivetrain;
  m_Xbox = &Xbox;
  m_intake = &intake;
  m_shooter = &shooter;
  m_arm = &arm;
  AddRequirements({m_limelight});
  AddRequirements({m_drivetrain});
  AddRequirements({m_intake});
  AddRequirements({m_shooter});
  AddRequirements({m_arm});
}

// Called when the command is initially scheduled.
void NoteFollower::Initialize()
{
  //  nt::NetworkTableInstance::GetDefault().GetTable("limelight-bac\k")->PutNumber("pipeline",0);
}

// Called repeatedly when this Command is scheduled to run
void NoteFollower::Execute() 
{
  m_intake->runIntake();
  m_intake->Direction();
  m_shooter->runMagazine();
  m_arm->runArmWheels();
  

  double tx = m_limelight->GetNotetx();
  if(tx > 7 || tx < -7){
    rot = units::angular_velocity::radians_per_second_t((0 - tx) * kp);
  }
  else
  {
    rot = units::angular_velocity::radians_per_second_t(0);
  }
  speedY = Deadzone(m_Xbox->GetLeftY());

  if((fabs(speedY) + fabs(rot.value())) < .05) 
  {
    NoJoystickInput = true;
  }
  else
  {
    NoJoystickInput = false;
  }

  m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY) * 6.7, units::velocity::meters_per_second_t(0), rot, false, NoJoystickInput);
}

// Called once the command ends or is interrupted.
void NoteFollower::End(bool interrupted) {}

// Returns true when the command should end.
bool NoteFollower::IsFinished()
{
   if(m_shooter->GetMagazineSensor()){
    m_intake->stopIntake();
    m_shooter->stopMagazine();
    m_arm->stopArmWheels();
    return true;
    
  }else{
    return false;
 }
}

float NoteFollower::Deadzone(float x)
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