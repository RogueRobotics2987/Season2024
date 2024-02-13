// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {
    LeftShooter.Follow(RightShooter, true);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
        ShooterActuator.Set((GetEncoderOffSet() - m_DesiredAngle) * kp); 

    frc::SmartDashboard::PutNumber("ShooterEncoder: ",GetEncoderOffSet());
}


void ShooterSubsystem::StopShooter(){
    RightShooter.Set(0.0);
}

void ShooterSubsystem::SetShooter(double speed) {
    RightShooter.Set(speed);
}

void ShooterSubsystem::ReverseShooter(){   
    RightShooter.Set(-0.2);
}


void ShooterSubsystem::SetActuator(double DesiredAngle) {
    m_DesiredAngle = DesiredAngle;
}   

bool ShooterSubsystem::GetMagazineSensor(){
    return MagazineSensor.Get();
}

bool ShooterSubsystem::IsTargeted(){
    return fabs(GetEncoderOffSet() - m_DesiredAngle) < ShooterConstants::AngleThreshold; 
}

double ShooterSubsystem::GetEncoderOffSet(){
    return ShooterEncoder.GetPosition() + ShooterConstants::EncoderOffSet;
}

void ShooterSubsystem::runMagazine(){
    MagazineMotor.Set(0.5);
}

void ShooterSubsystem::stopMagazine(){
    MagazineMotor.Set(0.0);
}
