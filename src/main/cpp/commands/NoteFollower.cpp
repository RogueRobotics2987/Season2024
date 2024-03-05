// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NoteFollower.h"

NoteFollower::NoteFollower(){}
NoteFollower::NoteFollower(LimelightSubsystem &limelight, DriveSubsystem &drivetrain, frc::XboxController &driverController)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_limelight = &limelight;
  m_drivetrain = &drivetrain;
  m_driverController = &driverController;

  AddRequirements({m_limelight});
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
frc::SmartDashboard::PutString("DriveConfiguration ", "NoteFollow");

    //limited drive, else regular
    if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tv", 0) == 1)
    {
      txNote = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx", 0.0);

      rotNote = units::angular_velocity::radians_per_second_t((0 - txNote) * kpNote);

      speedY = Deadzone(m_driverController->GetLeftY());

      if((fabs(speedY) + fabs(rotNote.value())) < .05)
      {
        NoJoystickInput = true;
      }
      else
      {
        NoJoystickInput = false;
      }
      m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY * AutoConstants::kMaxSpeed), units::velocity::meters_per_second_t(0), rotNote, false, NoJoystickInput);
    }   
    else
    {
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

        m_drivetrain->Drive(units::velocity::meters_per_second_t(-speedY * AutoConstants::kMaxSpeed), units::velocity::meters_per_second_t(-speedX * AutoConstants::kMaxSpeed), units::radians_per_second_t(-rot * AutoConstants::kMaxAngularSpeed), true, NoJoystickInput);
    }
}


// Called once the command ends or is interrupted.
void NoteFollower::End(bool interrupted) {}

// Returns true when the command should end.
bool NoteFollower::IsFinished(){}

float NoteFollower::Deadzone(float x)
{
  if ((x < 0.1) && (x > -0.1))
  {
    x=0;
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