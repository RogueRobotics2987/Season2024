// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {

    MagazineMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    MagazinePID.SetP(1);

}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {

    double angleError =  DistanceBetweenAngles(m_DesiredAngle, GetOffSetEncoderValue());

    if(DebugConstants::debug == true)
    {
        //MagazinePID.SetReference(shooterWheelsPos/1.4, rev::CANSparkMax::ControlType::kPosition);
        frc::SmartDashboard::PutNumber("goal magPos", shooterWheelsPos);
        frc::SmartDashboard::PutNumber("Actual Mag Pos", MagazineEncoder.GetPosition());
        frc::SmartDashboard::PutBoolean("ColorMag", MagazineSensor.Get());
        frc::SmartDashboard::PutNumber("ShooterEncoder: ",GetOffSetEncoderValue());
        frc::SmartDashboard::PutNumber("Raw Shooter Encoder", ShooterEncoder.GetAbsolutePosition());
        frc::SmartDashboard::PutNumber("ShooterDesired", m_DesiredAngle);
        frc::SmartDashboard::PutNumber("AngleTrim", angleTrim);
        // m_DesiredAngle = frc::SmartDashboard::GetNumber("SetAngle", m_DesiredAngle);
    }

    if(m_DesiredAngle >= ShooterConstants::RaisedShooterAngle)
    {
        m_DesiredAngle = ShooterConstants::RaisedShooterAngle;
    }
    else if(m_DesiredAngle <= ShooterConstants::RestingAngle)
    {
        m_DesiredAngle = ShooterConstants::RestingAngle;
    }

    double angleOutput = ((angleError * ShooterConstants::kp)) + accumulatedError;

    ShooterActuator.Set(-angleOutput); 

}

void ShooterSubsystem::JoystickActuator(double pos){
    if(fabs(pos) > .15){
        m_DesiredAngle += pos * 0.3;
    }
}

void ShooterSubsystem::StopShooter(){
    RightShooter.Set(0.0);
    LeftShooter.Set(0.0);
}

void ShooterSubsystem::SetShooter(double speedRight, double speedLeft) {
    RightShooter.Set(speedRight);
    LeftShooter.Set(speedLeft);
}

void ShooterSubsystem::ReverseShooter(){   
    RightShooter.Set(-0.2);
    LeftShooter.Set(0.2);
}


void ShooterSubsystem::SetActuator(double DesiredAngle) {
    double diff = m_DesiredAngle - DesiredAngle;
    shooterWheelsPos -= diff/132;

    m_DesiredAngle = DesiredAngle;
    //spinMag();
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

void ShooterSubsystem::runMagazine(double speed){
    MagazineMotor.Set(speed);
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
}

void ShooterSubsystem::SetIntakePose(){
    m_DesiredAngle = ShooterConstants::RestingAngle;
}

void ShooterSubsystem::ApriltagShooterTheta(double dist){
    m_DesiredAngle = (-3.45 * (dist * 3.28084)) + 66.3 + angleTrim; //the equation is in feet but our distance is in meters so we convert it to feet for the equation
}

void ShooterSubsystem::AngleTrimAdjust(bool buttonUp, bool buttonDown){
    if(buttonUp){
        angleTrim++;
    }
    else if(buttonDown){
        angleTrim--;
    }
}

void ShooterSubsystem::zeroIntergralVal(){
    accumulatedError = 0;
}

void ShooterSubsystem::accumulateError(){
    double angleError =  DistanceBetweenAngles(m_DesiredAngle, GetOffSetEncoderValue());

    if(accumulatedError < 0.15){
        accumulatedError += ShooterConstants::ki * angleError;
    }
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

void ShooterSubsystem::spinMag(){
    //MagazinePID.SetReference(shooterWheelsPos * 60, rev::CANSparkMax::ControlType::kPosition);
    //shooterWheelsPos += DistanceBetweenAngles(m_DesiredAngle, GetOffSetEncoderValue());
    //MagazinePID.SetReference(shooterWheelsPos * 0.1, rev::CANSparkMax::ControlType::kPosition);
    MagazineMotor.Set(DistanceBetweenAngles(m_DesiredAngle, GetOffSetEncoderValue()) * 0.035);
}

void ShooterSubsystem::SetMagPos(double val){
    if(m_DesiredAngle <= ShooterConstants::RaisedShooterAngle || m_DesiredAngle >= ShooterConstants::RestingAngle){
        shooterWheelsPos -= (val * 0.3)/132;
    }
        
    //spinMag();
}

double ShooterSubsystem::GetAngleError(){
    return DistanceBetweenAngles(m_DesiredAngle, GetOffSetEncoderValue());
}

void ShooterSubsystem::ResetMagEncoder(){
    MagazineEncoder.SetPosition(0.0);
    shooterWheelsPos = 0.0;
}