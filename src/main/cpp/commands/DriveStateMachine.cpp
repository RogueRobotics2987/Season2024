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
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline",0);

}

// Called repeatedly when this Command is scheduled to run
void DriveStateMachine::Execute() {
  frc::SmartDashboard::PutBoolean("runIntake - ds", runIntake);
  frc::SmartDashboard::PutBoolean("runShooterWarmup - ds", runShooterWarmup);
  
  //Buttons
  if(m_driverController->GetRawButtonPressed(5)){
    //noteFollowState = true;
    runIntake = !runIntake;
    //nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline",0);
    runShooterWarmup = false;
  } 

  if(m_driverController->GetRawButtonPressed(6)){
    runShooterWarmup = !runShooterWarmup;
    //nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline", 1);
    runIntake = false;
  }

  // if(m_driverController->GetRawAxis(3) > 0.05){
  //   standard = true;
  // }


  if(m_driverController->GetRawButtonPressed(1) && runIntake == true){
    noteFollowState = !noteFollowState;
  }

  if(m_driverController->GetRawButtonPressed(2) && runShooterWarmup == true){
    aprilFollowState = !aprilFollowState;
  }

  switch (drive_state) 
  {
    case NONE:
      frc::SmartDashboard::PutString("drive state", "NONE");

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

      m_drive->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(speedX), units::radians_per_second_t(rot), false, NoJoystickInput);

      if(noteFollowState == true){
        drive_state = NOTE_FOLLOW;
        standard = false;

      } else if(aprilFollowState == true){
        drive_state = APRIL_FOLLOW;
        standard = false;
      }

      break;

    case NOTE_FOLLOW:
      frc::SmartDashboard::PutString("drive state", "NOTE_FOLLOW");

      if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tv",0) == 1){

        txNote = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx",0.0);

        if(txNote > 7 || txNote < -7)
        {
          rotNote = units::angular_velocity::radians_per_second_t((0 + txNote) * kpNote);
        }
        else
        {
          rotNote = units::angular_velocity::radians_per_second_t(0);
        }

        speedY = Deadzone(m_driverController->GetLeftY());

        if((fabs(speedY) + fabs(rotNote.value())) < .05)
        {
          NoJoystickInput = true;
        }
        else
        {
          NoJoystickInput = false;
        }
        
        m_drive->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(0), rotNote, false, NoJoystickInput);

      } else {
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

        m_drive->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(speedX), units::radians_per_second_t(rot), false, NoJoystickInput);
      }
    
  
      if(m_messager->GetMessage().compare("Pickup") != 0)   // TODO: DOUBLE CHECK!!!
      {
        drive_state = NONE;
        noteFollowState = false;
      }

      if(noteFollowState == false){
         drive_state = NONE;
      }
      
      break;

    case APRIL_FOLLOW:
      frc::SmartDashboard::PutString("drive state", "APRIL_FOLLOW");
      
      if(m_limelight->PhotonHasTarget() == true){

        txApril = m_limelight->PhotonYaw(); //m_limelight->GetAprilTagtx() - 5; // TODO: check!

        //rotApril = units::angular_velocity::radians_per_second_t(0);
        // if(tx > 7 || tx < -7){
        rotApril = units::angular_velocity::radians_per_second_t((0-txApril) * kpApril);

        speedY = Deadzone(m_driverController->GetLeftY());
        speedX = Deadzone(m_driverController->GetLeftX());

        if((fabs(speedY) + fabs(speedX) + fabs(rotApril.value())) < .05)
        {
          NoJoystickInput = true;
        }
        else
        {
          NoJoystickInput = false;
        }
        
        m_drive->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(speedX), rotApril, false, NoJoystickInput);

      } else {
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

        m_drive->Drive(units::velocity::meters_per_second_t(speedY), units::velocity::meters_per_second_t(speedX), units::radians_per_second_t(rot), false, NoJoystickInput);
      }
    

      if(m_messager->GetMessage().compare("ShooterWarmup") != 0)
      {
        drive_state = NONE;
        aprilFollowState = false;
      }

      if(aprilFollowState == false){
        drive_state = NONE;
      }

      // if(standard == true){
      //   drive_state = NONE;
      //   aprilFollowState = false;
      // }

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