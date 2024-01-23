// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeShooterSubsystem.h"

IntakeSubsystem::IntakeSubsystem() = default;

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
    switch (state) {
    case EMPTY:
        /* code */
        break;
    
    case PICKUP:

        break;

    case MAGAZINE:

        break;

    case LOADED:

        break;

    case SHOOTER_WARMUP:

        break;

    case SHOOT:

        break;
    
    default:
        state = EMPTY;
        break;
    }

}
