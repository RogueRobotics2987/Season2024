// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {
    LeftShooter.Follow(RightShooter, true);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::StopShooter()
    {
        RightShooter.Set(0.0);
    }

void ShooterSubsystem::SetShooter() 
    {
        RightShooter.Set(1);
    }

void ShooterSubsystem::ReverseShooter()
    {   
        RightShooter.Set(-0.2);
    }

void ShooterSubsystem::SetActuator(double CurrentAngle, double DesiredAngle, double kp) 
    {
        ShooterActuator.Set((CurrentAngle - DesiredAngle) * kp); 
    }   