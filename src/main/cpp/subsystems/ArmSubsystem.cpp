// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem() {
    LowerArm = new rev::CANSparkMax(17, rev::CANSparkMax::MotorType::kBrushless);
    UpperArm = new rev::CANSparkMax(18, rev::CANSparkMax::MotorType::kBrushless);
    ArmWheels = new rev::CANSparkMax(19, rev::CANSparkMax::MotorType::kBrushless);
}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {}
