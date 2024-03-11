// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpCommand.h"

AmpCommand::AmpCommand() {}
AmpCommand::AmpCommand(ArmSubsystem &arm)
{
  m_arm = &arm;
  AddRequirements({m_arm});
}

// Called when the command is initially scheduled.
void AmpCommand::Initialize()
{
  m_arm->SetLowerArmAngle(112);
}

// Called repeatedly when this Command is scheduled to run
void AmpCommand::Execute()
{
  if(m_arm->GetOffSetEncoderValueLower() > 100)
  {
    m_arm->RunArmWheels(-1);
  }
}

// Called once the command ends or is interrupted.
void AmpCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AmpCommand::IsFinished()
{
  return false;
}
