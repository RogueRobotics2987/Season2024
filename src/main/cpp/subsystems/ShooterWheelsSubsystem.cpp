// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterWheelsSubsystem.h"

ShooterWheelsSubsystem::ShooterWheelsSubsystem() = default;

// This method will be called once per scheduler run
void ShooterWheelsSubsystem::Periodic()
{

}

void ShooterWheelsSubsystem::SetShooter(double speedBottom, double speedTop) {
    BottomShooter.Set(speedBottom);
    TopShooter.Set(speedTop);
}

void ShooterWheelsSubsystem::ReverseShooter(){   
    BottomShooter.Set(-0.2);
    TopShooter.Set(0.2);
}

void ShooterWheelsSubsystem::StopShooter(){
    TopShooter.Set(0.0);
    BottomShooter.Set(0.0);
}