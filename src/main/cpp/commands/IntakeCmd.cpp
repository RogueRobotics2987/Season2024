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
  m_intake->runIntake(0.25);
  m_intake->Direction(0.25); //possibly up all these speeds
  m_intake->runMagazine(0.25);
  state = 0;
  time = 0;
}

// Called repeatedly when this Command is scheduled to run
void IntakeCmd::Execute()
{
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
    m_intake->runIntake(0);
    m_intake->Direction(0);
    m_intake->runMagazine(0);

    state = 2;
  }
  else if(state == 2)
  {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->PutNumber("pipeline",1);
    camera.SetPipelineIndex(1);

    time++;
    m_intake->runMagazine(-0.2);

    if(time <= 7)
    {
      finished = true;
    }
  }
}

// Called once the command ends or is interrupted.
void IntakeCmd::End(bool interrupted)
{
  m_intake->runIntake(0);
  m_intake->Direction(0);
  m_intake->runMagazine(0);
}

// Returns true when the command should end.
bool IntakeCmd::IsFinished()
{
  return finished;
}
