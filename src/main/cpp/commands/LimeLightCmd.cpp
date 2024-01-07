// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LimeLightCmd.h"


LimeLightCmd::LimeLightCmd(){}
LimeLightCmd::LimeLightCmd(LimelightPose &limePose) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_limePose = &limePose;
  AddRequirements({m_limePose});
}

// Called when the command is initially scheduled.
void LimeLightCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LimeLightCmd::Execute() {
    //TODO output vector "botpose" from limelight
    // std::cout << botPose[0] << std::endl;
    // std::cout << botPose[1] << std::endl;
    // std::cout << botPose[2] << std::endl;

}

// Called once the command ends or is interrupted.
void LimeLightCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool LimeLightCmd::IsFinished() {
  return false;
}
