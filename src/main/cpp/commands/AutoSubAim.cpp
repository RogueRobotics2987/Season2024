// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoSubAim.h"

AutoSubAim::AutoSubAim() {}

AutoSubAim::AutoSubAim(ShooterSubsystem &shooter) {
  m_shooter = &shooter;
  AddRequirements({m_shooter});
}

// Called when the command is initially scheduled.
void AutoSubAim::Initialize() {
  m_shooter->SetActuator(ShooterConstants::SubwooferAngle); 
}

// Called repeatedly when this Command is scheduled to run
void AutoSubAim::Execute() {
  if(time >= 60)
  {
    timeIsUp = true;
  }
  else
  {
    time++;
  }
}

// Called once the command ends or is interrupted.
void AutoSubAim::End(bool interrupted) {
  m_shooter->setRestingActuatorPosition();
}

// Returns true when the command should end.
bool AutoSubAim::IsFinished() {
  return timeIsUp;
}
