// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ManualAim.h"

ManualAim::ManualAim(ShooterSubsystem &shooter, frc::XboxController &auxController)
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_shooter = &shooter;
  m_auxController = &auxController;

  AddRequirements({m_shooter});
}

// Called when the command is initially scheduled.
void ManualAim::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ManualAim::Execute()
{
  m_shooter->AngleTrimAdjust(m_auxController->GetRawButtonPressed(6), m_auxController->GetRawButtonPressed(5));
  m_shooter->JoystickActuator(m_auxController->GetRightY());
}

// Called once the command ends or is interrupted.
void ManualAim::End(bool interrupted) {}

// Returns true when the command should end.
bool ManualAim::IsFinished() 
{
  return false;
}
