// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem()
{
    ClimberMoter = new rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless);
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
