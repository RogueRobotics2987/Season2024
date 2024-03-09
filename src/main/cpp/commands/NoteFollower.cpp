// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NoteFollower.h"

NoteFollower::NoteFollower(){}
NoteFollower::NoteFollower(LimelightSubsystem &limelight, DriveSubsystem &drivetrain, frc::XboxController &driverController, IntakeSubsystem &intake)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_limelight = &limelight;
  m_intake = &intake;
  m_drivetrain = &drivetrain;
  m_driverController = &driverController;

  AddRequirements({m_intake});
  AddRequirements({m_limelight});
  AddRequirements({m_drivetrain});
}

// Called when the command is initially scheduled.
void NoteFollower::Initialize()
{
  //  nt::NetworkTableInstance::GetDefault().GetTable("limelight-bac\k")->PutNumber("pipeline",0);
  m_intake->runIntake(0.3);
  m_intake->DirectionNote(0.25); //possibly up all these speeds
  m_intake->runMagazine(0.25);
  state = 0;
  time = 0;
  finished = false;
}

// Called repeatedly when this Command is scheduled to run
void NoteFollower::Execute() 
{
  frc::SmartDashboard::PutBoolean("noteFollower", true);

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

        m_drivetrain->Drive(
          units::velocity::meters_per_second_t(speedY * AutoConstants::kMaxSpeed),
          units::velocity::meters_per_second_t(-speedX * AutoConstants::kMaxSpeed),
          units::radians_per_second_t(-rot * AutoConstants::kMaxAngularSpeed),
          false,
          NoJoystickInput
        );
    }

  if(state == 0)
  {
    //just run until we have a note

    if(m_intake->GetMagazineSensor())
    {
      state = 1;
    }
  }
  else if(state == 1)
  {
    m_intake->stopIntake();
    m_intake->stopMagazine();

    state = 2;
  }
  else if(state == 2)
  {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline",1);

    time++;
    m_intake->runMagazine(-0.2);

    if(time >= 7)
    {
      finished = true;
    }
  }
}


// Called once the command ends or is interrupted.
void NoteFollower::End(bool interrupted)
{
  frc::SmartDashboard::PutBoolean("noteFollower", true);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline",0);
}

// Returns true when the command should end.
bool NoteFollower::IsFinished()
{
  return finished;
}

float NoteFollower::Deadzone(float x)
{
  if ((x < 0.1) && (x > -0.1))
  {
    x = 0;
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