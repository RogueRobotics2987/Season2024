// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NoteFollower.h"

NoteFollower::NoteFollower(){}
NoteFollower::NoteFollower(LimelightSubsystem &limelight, DriveSubsystem &drivetrain, 
                           frc::XboxController &driverController, IntakeSubsystem &intake, LightSubsystem &lights, frc::XboxController &auxController)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_limelight = &limelight;
  m_intake = &intake;
  m_drivetrain = &drivetrain;
  m_driverController = &driverController;
  m_lights = &lights;
  m_auxController = &auxController;

  AddRequirements({m_intake});
  AddRequirements({m_limelight});
  AddRequirements({m_drivetrain});
  AddRequirements({m_lights});
}

// Called when the command is initially scheduled.
void NoteFollower::Initialize()
{
  //  nt::NetworkTableInstance::GetDefault().GetTable("limelight-bac\k")->PutNumber("pipeline",0);
  m_intake->RunIntake(0.5);
  m_intake->DirectionNote(0.45); //possibly up all these speeds
  m_intake->RunMagazine(0.5);
  state = 0;
  time = 0;
  finished = false;
  noteDrive = false;
  hasNote = false;

  m_lights->SetLightsYellow();

  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline",0);

  frc::SmartDashboard::PutBoolean("NormalDrive", false);
  frc::SmartDashboard::PutBoolean("NoteDrive", false);
}

// Called repeatedly when this Command is scheduled to run
void NoteFollower::Execute() 
{
  frc::SmartDashboard::PutBoolean("noteFollower", true);

  //limited drive, else regular
  if(noteDrive == true)
  {
    frc::SmartDashboard::PutBoolean("NoteDrive", true);

    txNote = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx", 0.0);
    tyNote = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("ty", 0.0);

    //noteError = (((21.86 + tyNote) /-8.369) - txNote);

    //noteError = (((21.18 + tyNote) /-6.8454) - txNote);

    noteError = (((15 + tyNote) /-8.369) - txNote);

    frc::SmartDashboard::PutNumber("NoteTrackerError", noteError);

    rotNote = units::angular_velocity::radians_per_second_t(noteError * kpNote);
    speedY = Deadzone(m_driverController->GetLeftY());

    if((fabs(speedY) + fabs(rotNote.value())) < .05)
    {
      NoJoystickInput = true;
    }
    else
    {
      NoJoystickInput = false;
    }

    if(fabs(noteError) > 5)
    {
      m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY * (AutoConstants::kMaxSpeed * 0.50)), units::velocity::meters_per_second_t(0), rotNote, false, NoJoystickInput);
    }
    else
    {
      m_drivetrain->Drive(units::velocity::meters_per_second_t(speedY * (AutoConstants::kMaxSpeed * 0.80)), units::velocity::meters_per_second_t(0), rotNote, false, NoJoystickInput);
    }
  }   
  else
  {

    frc::SmartDashboard::PutBoolean("NoteDrive", false);
    frc::SmartDashboard::PutBoolean("NormalDrive", true);

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

    if(nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tv", 0) != 0 && hasNote == false)
    {
      noteDrive = true;
      std::cout<< "Line 94" << std::endl;
    }
  }

  if(state == 0)
  {
    //just run until we have a note
    time++;

    if(noteDrive == true && m_intake->GetBackIntakeCurrent() > 25 && time > 40)
    {
      noteDrive = false;
      hasNote = true;
    }

    if(m_intake->GetMagazineSensor())
    {
      state = 1;
      m_lights->SetLightsGreen();
      m_driverController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1);
      m_auxController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1);

      time = 0;
    }
  }
  else if(state == 1)
  {
    m_intake->StopIntake();
    m_intake->StopMagazine();

    state = 2;
  }
  else if(state == 2)
  {
    time++;
    m_intake->RunMagazine(-0.2);

    if(m_intake->GetMagazineSensor() && time >= 10)
    {
      finished = true;
    }
  }
}

// Called once the command ends or is interrupted.
void NoteFollower::End(bool interrupted)
{
  frc::SmartDashboard::PutBoolean("noteFollower", true);

  if(state == 2)
  {
    m_lights->SetLightsGreen();
  }
  else
  {
    m_lights->SetNoColor();
    //m_lights->SetColorChase();
  }

  m_driverController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
  m_auxController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
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