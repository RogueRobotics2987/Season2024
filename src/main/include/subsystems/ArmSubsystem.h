// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "rev/AbsoluteEncoder.h"
#include <Constants.h> 
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/DutyCycleEncoder.h>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

class ArmSubsystem : public frc2::SubsystemBase
{
  public:
    ArmSubsystem();
    double GetOffSetEncoderValueLower();
    double GetOffSetEncoderValueUpper();
  
    // DO NOT SET TO ZERO doesnt acutally update....
    void SetLowerArmAngle(double desiredAngle);

    // void setVoltage(double speed);
    void AccumulateErrorLower();

    void RunLowerArm();

    void ZeroIntergral();

    void StopWheels();

    void RunArmWheels(double speed);
    void StopArmWheels();
    void FollowShooter(double error);

    void MoveLowerArm();
    double GetLowerArmError();
    double GetLowerEncoderPos();  
    double DistanceBetweenAngles(double targetAngle, double sourceAngle);

    void Periodic() override;

  private:
    rev::CANSparkMax LowerArm{17, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax ArmWheels{19, rev::CANSparkMax::MotorType::kBrushless};

    frc::DutyCycleEncoder m_LowerArmEncoder{7}; //lower encoder value degrees = 30, lower encode valuse degrees extended = 112
    double LoweraccumulatedError = 0;
    double m_LowerDesired = 0;
};
