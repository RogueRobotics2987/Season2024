// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCmd.h"

IntakeCmd::IntakeCmd(IntakeSubsystem &intake, LightSubsystem &light, frc::XboxController &driverController, frc::XboxController &auxController)
{
  m_intake = &intake;
  m_lights = &light;
  m_driverController = &driverController;
  m_auxController = &auxController;

  AddRequirements(m_intake);
  AddRequirements(m_lights);
}

// Called when the command is initially scheduled.
void IntakeCmd::Initialize()
{
  m_intake->RunIntake(0.5);
  m_intake->DirectionNote(0.45); //possibly up all these speeds
  m_intake->RunMagazine(0.5);
  state = 0;
  time = 0;
  finished = false;
  frc::SmartDashboard::PutBoolean("IntakeON", true);
  m_lights->SetLightsYellow();
}

// Called repeatedly when this Command is scheduled to run
void IntakeCmd::Execute()
{
  m_intake->Direction(0.35);

  if(state == 0)
  {
    //just run until we have a note
    if(m_intake->GetMagazineSensor())
    {
      state = 1;
      m_lights->SetLightsGreen();
      m_driverController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1);
      m_auxController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1);
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
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline",1);

    time++;
    m_intake->RunMagazine(-0.2);

    if(m_intake->GetMagazineSensor() && time >= 10)
    {
      finished = true;
    }
  }
}

// Called once the command ends or is interrupted.
void IntakeCmd::End(bool interrupted)
{
  if(state == 2)
  {
    m_lights->SetLightsGreen();
  }
  else
  {
    m_lights->SetNoColor();
  }

  m_driverController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
  m_auxController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
  m_intake->RunIntake(0);
  m_intake->Direction(0);
  m_intake->RunMagazine(0);
  frc::SmartDashboard::PutBoolean("IntakeON", false);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline",0);
}

// Returns true when the command should end.
bool IntakeCmd::IsFinished()
{
  return finished;
}
