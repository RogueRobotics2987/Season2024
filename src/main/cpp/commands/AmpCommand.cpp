// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpCommand.h"

AmpCommand::AmpCommand(){}
AmpCommand::AmpCommand(ArmSubsystem &arm)
{
  m_arm = &arm;
  AddRequirements({m_arm});
}


// Called when the command is initially scheduled.
void AmpCommand::Initialize()
{
  m_arm->setLowerArmAngle(112);
}

// Called repeatedly when this Command is scheduled to run
void AmpCommand::Execute()
{
  frc::SmartDashboard::PutString("armAmp", "");
}

// Called once the command ends or is interrupted.
void AmpCommand::End(bool interrupted)
{
  m_arm->setLowerArmAngle(30);
  std::cout << "should be 30" << std::endl;
}

// Returns true when the command should end.
bool AmpCommand::IsFinished()
{
  return false;
}
