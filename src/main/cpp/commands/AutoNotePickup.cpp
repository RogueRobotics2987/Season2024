// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoNotePickup.h"

AutoNotePickup::AutoNotePickup(){}
AutoNotePickup::AutoNotePickup(LimelightSubsystem &limelight, DriveSubsystem &drivetrain, IntakeSubsystem &intake, LightSubsystem &light)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_limelight = &limelight;
  m_intake = &intake;
  m_drivetrain = &drivetrain;
  m_lights = &light;

  AddRequirements({m_lights});
  AddRequirements({m_intake});
  AddRequirements({m_limelight});
  AddRequirements({m_drivetrain});
}

// Called when the command is initially scheduled.
void AutoNotePickup::Initialize()
{
  //  nt::NetworkTableInstance::GetDefault().GetTable("limelight-bac\k")->PutNumber("pipeline",0);
  m_intake->RunIntake(0.5);
  m_intake->DirectionNote(0.45); //possibly up all these speeds
  m_intake->RunMagazine(0.5);
  state = 0;
  time = 0;
  finished = false;

  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline",0);
}

// Called repeatedly when this Command is scheduled to run
void AutoNotePickup::Execute() 
{
  frc::SmartDashboard::PutBoolean("AutoNotePickup", true);
  
  if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tv", 0) > 0)
  {
    txNote = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx", 0.0);
    tyNote = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("ty", 0.0);
  }

  noteError = (((15 + tyNote) /-8.369) - txNote);

  frc::SmartDashboard::PutNumber("NoteTrackerError", noteError);

  rotNote = units::angular_velocity::radians_per_second_t(noteError * kpNote);

  // if(fabs(rotNote.value()) < .05)
  // {
  //   NoInput = true;
  // }
  // else
  // {
  //   NoInput = false;
  // }

  if(state == 0)
  {
    if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tv", 0) > 0)
    {
      direction = -1;
      m_lights->SetLightsPurple();
    }
    else
    {
      direction = 1;
      m_lights->SetLightsYellow();
    }

    if(fabs(noteError) > 5)
    {
      m_drivetrain->Drive(direction * (driveSpeed * 0.50), units::velocity::meters_per_second_t(0), rotNote, false, false);
    }
    else
    {
      m_drivetrain->Drive(direction * (driveSpeed * 0.80), units::velocity::meters_per_second_t(0), rotNote, false, false);
    }

    if(m_intake->GetMagazineSensor())
    {
      state = 1;
      m_drivetrain->Drive(1_mps, 0_mps, 0_rad_per_s, false, false);
    }
  }
  else if(state == 1)
  {
    m_intake->StopIntake();
    m_intake->StopMagazine();

    m_lights->SetLightsGreen();

    state = 2;
  }
  else if(state == 2)
  {
    time++;
    m_intake->RunMagazine(-0.2);

    if(time >= 10)
    {
      finished = true;
    }
  }
}

// Called once the command ends or is interrupted.
void AutoNotePickup::End(bool interrupted)
{
  frc::SmartDashboard::PutBoolean("AutoNotePickup", true);

  if(finished == true)
  {
    m_lights->SetLightsGreen();
  }
  else
  {
    m_lights->SetNoColor();
  }
  
}

// Returns true when the command should end.
bool AutoNotePickup::IsFinished()
{
  return finished;
}
