// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {
    LeftShooter = new rev::CANSparkMax(15, rev::CANSparkMax::MotorType::kBrushless);
    RightShooter = new rev::CANSparkMax(16, rev::CANSparkMax::MotorType::kBrushless);
    ShooterActuator = new rev::CANSparkMax(13, rev::CANSparkMax::MotorType::kBrushless);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}
