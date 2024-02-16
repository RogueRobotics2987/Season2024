// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {
    LeftShooter.Follow(RightShooter, true);
    frc::SmartDashboard::PutNumber("ShooterDesired", m_DesiredAngle);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
    if(DebugConstants::debug == true){
        frc::SmartDashboard::PutBoolean("ColorMag", MagazineSensor.Get());
        frc::SmartDashboard::PutNumber("ShooterEncoder: ",GetOffSetEncoderValue());
        frc::SmartDashboard::PutNumber("Raw Shooter Encoder", ShooterEncoder.GetAbsolutePosition());
        frc::SmartDashboard::GetNumber("ShooterDesired", m_DesiredAngle);
        //TODO test that shooter will update and move to desired angle from smart dashboard
    }
}

void ShooterSubsystem::JoystickActuator(double pos){
    if(fabs(pos) > .15){
        m_DesiredAngle += pos*.1;
    }
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
    return fabs(GetOffSetEncoderValue() - m_DesiredAngle) < ShooterConstants::AngleThreshold; 
}

//Calculates encoder position and changes it to degrees
double ShooterSubsystem::GetOffSetEncoderValue(){
    double Pose = 0;
    Pose = ShooterEncoder.GetAbsolutePosition() + ShooterConstants::EncoderOffSet;

    if(Pose < 0){
        Pose += 1;
    }

    Pose = fabs(Pose - 1);
    Pose *= 132;

    return Pose;
}

void ShooterSubsystem::runMagazine(){
    MagazineMotor.Set(0.5);
}

void ShooterSubsystem::stopMagazine(){
    MagazineMotor.Set(0.0);
}

void ShooterSubsystem::driveActuator(double speed){
    if(speed > 0.1){
        ShooterActuator.Set(0.05);
    }
    else if(speed < -0.1){
        ShooterActuator.Set(-0.05);
    }
    else{
        ShooterActuator.Set(0.0);
    }
}

void ShooterSubsystem::setRestingActuatorPosition(){
    m_DesiredAngle = ShooterConstants::RestingAngle; 
    //the -1 Flips polarity so motor will move the correct way
    ShooterActuator.Set((DistanceBetweenAngles(m_DesiredAngle, GetOffSetEncoderValue()) * kp) * -1); 
}

double ShooterSubsystem::DistanceBetweenAngles(double targetAngle, double sourceAngle)
{
  double a = targetAngle - sourceAngle;
  if(a > 180)
  {
    a = a + -360;
  }
  else if(a < -180)
  {
    a = a + 360;
  }
  else
  {
    a = a;
  }

  return a;
}

