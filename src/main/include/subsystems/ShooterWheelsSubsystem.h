// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "rev/SparkMaxRelativeEncoder.h"
#include "ctre/Phoenix.h"
#include <Constants.h>
#include "networktables/NetworkTableInstance.inc"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <iostream>

class ShooterWheelsSubsystem : public frc2::SubsystemBase
{
  public:
    ShooterWheelsSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    void StopShooter();
    void SetShooter(double speedRight, double speedLeft);
    void ReverseShooter();
    void PIDShoot();

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    rev::CANSparkMax TopShooter{15, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax BottomShooter{16, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkPIDController TopShooterPID = TopShooter.GetPIDController();
    rev::SparkPIDController BottomShooterPID = BottomShooter.GetPIDController();
    rev::SparkRelativeEncoder TopShooterEncoder = TopShooter.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkRelativeEncoder BottomShooterEncoder = BottomShooter.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
};
