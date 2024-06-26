// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() 
{
    ShooterActuator.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
    ShooterActuator.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    ShooterActuator.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus4, 500);
    ShooterActuator.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 500);
    ShooterActuator.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus6, 500);
    sleep(1); 
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic()
{
    //tempKp = frc::SmartDashboard::GetNumber("shooter actuator kp", tempKp);

    if(DebugConstants::debugShooter == true)
    {
        // frc::SmartDashboard::PutBoolean("ColorMag", MagazineSensor.Get());
        frc::SmartDashboard::PutNumber("ShooterEncoder: ",GetOffSetEncoderValue());
        frc::SmartDashboard::PutNumber("Raw Shooter Encoder", ShooterEncoder.GetAbsolutePosition());
        frc::SmartDashboard::PutNumber("ShooterDesired", m_DesiredAngle);
        frc::SmartDashboard::PutNumber("AngleTrim", angleTrim);
        // m_DesiredAngle = frc::SmartDashboard::GetNumber("SetAngle", m_DesiredAngle);
    }
    SetShooterAngle();
}

void ShooterSubsystem::JoystickActuator(double pos)
{
    if(fabs(pos) > .15)
    {
        m_DesiredAngle += pos*.3;
    }
}

void ShooterSubsystem::SetActuator(double DesiredAngle) {
    m_DesiredAngle = DesiredAngle;
}   

bool ShooterSubsystem::IsTargeted(){
    return fabs(GetOffSetEncoderValue() - m_DesiredAngle) < ShooterConstants::AngleThreshold; 
}

//Calculates encoder position and changes it to degrees
double ShooterSubsystem::GetOffSetEncoderValue()
{
    double Pose = 0;
    Pose = ShooterEncoder.GetAbsolutePosition() + ShooterConstants::EncoderOffSet;

    if(Pose < 0)
    {
        Pose += 1;
    }

    Pose = fabs(Pose - 1);
    Pose *= 132;

    return Pose;
}

double ShooterSubsystem::ShooterError()
{
    return DistanceBetweenAngles(m_DesiredAngle, GetOffSetEncoderValue());
}

void ShooterSubsystem::driveActuator(double speed)
{
    if(speed > 0.1)
    {
        ShooterActuator.Set(0.05);
    }
    else if(speed < -0.1)
    {
        ShooterActuator.Set(-0.05);
    }
    else
    {
        ShooterActuator.Set(0.0);
    }
}

//TODO: WHY ARE THESE TWO FUNCTIONS DOING THE SAME THING
void ShooterSubsystem::setRestingActuatorPosition()
{
    m_DesiredAngle = ShooterConstants::RestingAngle; 
}

void ShooterSubsystem::SetIntakePose()
{
    m_DesiredAngle = ShooterConstants::RestingAngle;
}

void ShooterSubsystem::AngleTrimAdjust(bool buttonUp, bool buttonDown)
{
    if(buttonUp)
    {
        angleTrim = angleTrim - 0.25;
    }
    else if(buttonDown)
    {
        angleTrim= angleTrim + 0.25;
    }
}

void ShooterSubsystem::zeroIntergralVal()
{
    accumulatedError = 0;
}

void ShooterSubsystem::accumulateError()
{
    double angleError =  DistanceBetweenAngles(m_DesiredAngle, GetOffSetEncoderValue());

    if(accumulatedError < 0.15)
    {
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

void ShooterSubsystem::SetShooterAngle()
{
    if(m_DesiredAngle >= ShooterConstants::ShooterMaxSoftLimit)
    {
        m_DesiredAngle = ShooterConstants::ShooterMaxSoftLimit;
    }
    else if(m_DesiredAngle <= ShooterConstants::ShooterMinSoftLimit)
    {
        m_DesiredAngle = ShooterConstants::ShooterMinSoftLimit;
    }

    double angleError = DistanceBetweenAngles(m_DesiredAngle, GetOffSetEncoderValue());
    double angleOutput = ((angleError * ShooterConstants::kp)) + accumulatedError;
    ShooterActuator.Set(-angleOutput); 
}

double ShooterSubsystem::GetDesired()
{
    return m_DesiredAngle;
}

double ShooterSubsystem::GetAngleTrim()
{
    return angleTrim;
}