// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCmd.h"

IntakeCmd::IntakeCmd(IntakeSubsystem &intake)
{
  m_intake = &intake;
  AddRequirements(m_intake);
}

// Called when the command is initially scheduled.
void IntakeCmd::Initialize()
{
  m_intake->RunIntake(0.3);
  m_intake->DirectionNote(0.25); //possibly up all these speeds
  m_intake->RunMagazine(0.25);
  state = 0;
  time = 0;
  finished = false;
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

    if(time <= 7)
    {
      finished = true;
    }
  }
}

// Called once the command ends or is interrupted.
void IntakeCmd::End(bool interrupted)
{
  m_intake->RunIntake(0);
  m_intake->Direction(0);
  m_intake->RunMagazine(0);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("pipeline",0);
}

// Returns true when the command should end.
bool IntakeCmd::IsFinished()
{
  return finished;
}
