// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() {
    BackIntake = new rev::CANSparkMax(9, rev::CANSparkMax::MotorType::kBrushless);
    FrontIntake = new rev::CANSparkMax(10, rev::CANSparkMax::MotorType::kBrushless);
    CenterIntake = new rev::CANSparkMax(11, rev::CANSparkMax::MotorType::kBrushless);
    Magazine = new rev::CANSparkMax(14, rev::CANSparkMax::MotorType::kBrushless);

}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}
