// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #include <frc2/command/Commands.h>
// #include <frc2/command/CommandHelper.h>
// #include <frc2/command/CommandPtr.h>
// #include <frc2/command/InstantCommand.h>
// #include <frc2/command/RunCommand.h>

#include <string>

class MessengerCommand {
 public:
  MessengerCommand();

  void SetAuxMessage(std::string message);
  std::string GetAuxMessage();

  void SetDriveMessage(std::string message);
  std::string GetDriveMessage();

 private:
  std::string DriveMessage = "initialDrive";  
  std::string AuxMessage = "initialAux";
};