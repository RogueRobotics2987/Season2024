// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "rev/SparkMaxRelativeEncoder.h"
#include "ctre/Phoenix.h"
#include <frc/DigitalInput.h>
#include <Constants.h>
#include "networktables/NetworkTableInstance.inc"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <math.h>
#include <iostream>

class ShooterSubsystem : public frc2::SubsystemBase
{
  public:
    ShooterSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    void JoystickActuator(double pos);

    void StopShooter();
    void SetShooter(double speedRight, double speedLeft);

    void ReverseShooter();
    
    void SetActuator(double DesiredAngle);

    bool GetMagazineSensor();
    
    bool IsTargeted();

    double GetOffSetEncoderValue();

    double ShooterError();
    void driveActuator(double speed);
    void setRestingActuatorPosition();
    double DistanceBetweenAngles(double targetAngle, double sourceAngle);
    void SetIntakePose();

    void ApriltagShooterTheta(double dist, double pos);
    void AngleTrimAdjust(bool buttonUp, bool buttonDown);
    void zeroIntergralVal();
    void accumulateError();
    void SetShooterAngle();
    double GetDesired();

    double GetAngleTrim();

  private:
    rev::CANSparkMax ShooterActuator{13, rev::CANSparkMax::MotorType::kBrushless};
    frc::DutyCycleEncoder ShooterEncoder{8};

    double m_DesiredAngle = 40; 
    double testAngle = 40;

    double angleTrim = 0;
    double accumulatedError = 0;
};