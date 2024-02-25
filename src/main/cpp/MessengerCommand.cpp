// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "MessengerCommand.h"

MessengerCommand::MessengerCommand() = default;

void MessengerCommand::SetAuxMessage(std::string message)
{
    AuxMessage = message;
}

std::string MessengerCommand::GetAuxMessage()
{
    return AuxMessage;
}

void MessengerCommand::SetDriveMessage(std::string message)
{  
    DriveMessage = message;
}

std::string MessengerCommand::GetDriveMessage()
{
    return DriveMessage;
}
