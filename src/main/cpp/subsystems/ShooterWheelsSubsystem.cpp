// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterWheelsSubsystem.h"

ShooterWheelsSubsystem::ShooterWheelsSubsystem()
{
    TopShooterPID.SetP(0.0005);
    BottomShooterPID.SetP(0.0005);
    TopShooterPID.SetFF(0.7 /3500);
    BottomShooterPID.SetFF(0.7 /3500);
};

// This method will be called once per scheduler run
void ShooterWheelsSubsystem::Periodic()
{
    if(DebugConstants::debugShooter == true)
    {
        frc::SmartDashboard::PutNumber("TopShooterVelocity", TopShooterEncoder.GetVelocity());
        frc::SmartDashboard::PutNumber("BottomShooterVelocity", BottomShooterEncoder.GetVelocity());
    }
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

void ShooterWheelsSubsystem::PIDShoot()
{
    TopShooterPID.SetReference(4062, rev::CANSparkMax::ControlType::kVelocity);
    BottomShooterPID.SetReference(4171, rev::CANSparkMax::ControlType::kVelocity);
}