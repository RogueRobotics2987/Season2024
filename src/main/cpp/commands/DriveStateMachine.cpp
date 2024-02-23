// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveStateMachine.h"

DriveStateMachine::DriveStateMachine() {}
  
DriveStateMachine::DriveStateMachine(DriveSubsystem &drive, LimelightSubsystem &limelight, frc::XboxController &driveXbox, frc::XboxController &auxXbox, CommandMessenger &message) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drive = &drive;
  AddRequirements(m_drive);
  m_limelight = &limelight;
  AddRequirements(m_limelight);
  m_messager = &message;

  m_driverController = &driveXbox;
  m_auxController = &auxXbox;
}

// Called when the command is initially scheduled.
void DriveStateMachine::Initialize() {
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->PutNumber("pipeline",1);

}

// Called repeatedly when this Command is scheduled to run
void DriveStateMachine::Execute() {
  //Buttons
  if(m_driverController->GetRawButtonPressed(5)){
    //noteFollowState = true;
    runIntake = true;
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->PutNumber("pipeline",0);
    runShooterWarmup = false;
  } 

  if(m_driverController->GetRawButtonPressed(6)){
    runShooterWarmup = true;
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->PutNumber("pipeline", 1);
    runIntake = false;
  }

  if(m_driverController->GetRawAxis(3) > 0.05){
    standard = true;
  }


  if(m_driverController->GetRawButtonPressed(1) && runIntake == true){
    if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tv",0) ==1){
      noteFollowState = true;
    }
  }

  if(m_driverController->GetRawButtonPressed(2) && runShooterWarmup == true){
    if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tv",0) ==1){
      aprilFollowState = true;
    }
  }

  switch (drive_state) 
  {
    case NONE:
      // not sure what should happen

      if(noteFollowState == true){
        drive_state = NOTE_FOLLOW;
        standard = false;

      } else if(aprilFollowState == true){
        drive_state = APRIL_FOLLOW;
        standard = false;
      }

      break;

    case NOTE_FOLLOW:
      txNote = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tx",0.0);

      if(m_messager->GetMessage().compare("Pickup") != 0)   // TODO: DOUBLE CHECK!!!
      {
        drive_state = NONE;
      }

      if(txNote > 7 || txNote < -7)
      {
        rotNote = units::angular_velocity::radians_per_second_t((0 + txNote) * kpNote);
      }
      else
      {
        rotNote = units::angular_velocity::radians_per_second_t(0);
      }
      
      m_drive->Drive(units::velocity::meters_per_second_t(m_driverController->GetLeftY()), units::velocity::meters_per_second_t(0), rotNote, false, false);


      if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tv",0) == 0){
        drive_state = NONE;
        noteFollowState = false;
      }
      
      break;

    case APRIL_FOLLOW:
      txApril = m_limelight->GetAprilTagtx() - 5;   // what is this number?

    if(m_messager->GetMessage().compare("Loaded") != 0 || m_messager->GetMessage().compare("ShooterWarmup"))  // TODO: oduble check!!!
      {
        drive_state = NONE;
      }

      //rotApril = units::angular_velocity::radians_per_second_t(0);
      // if(tx > 7 || tx < -7){
      rotApril = units::angular_velocity::radians_per_second_t((0-txApril) * kpApril);

      speedY = Deadzone(m_driverController->GetLeftY());
      speedX = Deadzone(m_driverController->GetLeftY());

      if((fabs(speedY) + fabs(speedX) + fabs(rotApril.value())) < .05)
      {
        NoJoystickInput = true;
      }
      else
      {
        NoJoystickInput = false;
      }
      
      m_drive->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(speedX), rotApril, false, NoJoystickInput);


      if(standard == true){
        drive_state = NONE;
        aprilFollowState = false;
      }

      break;

    default:
      drive_state = NONE;
      break;
  }

}

// Called once the command ends or is interrupted.
void DriveStateMachine::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveStateMachine::IsFinished() {
  return false;
}


float DriveStateMachine::Deadzone(float x)
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