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

class ShooterSubsystem : public frc2::SubsystemBase {
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

  void runMagazine(double speed);
  void stopMagazine();
  void holdMagazine(double pos);
  double GetCurrMagEncoderVal();

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

  void PIDShoot();


 private:
  rev::CANSparkMax TopShooter{15, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax BottomShooter{16, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax ShooterActuator{13, rev::CANSparkMax::MotorType::kBrushless};
  //rev::SparkMaxAlternateEncoder ShooterEncoder{ShooterActuator.GetAlternateEncoder(8192)};
  frc::DutyCycleEncoder ShooterEncoder{8};
  rev::SparkPIDController TopShooterPID = TopShooter.GetPIDController();
  rev::SparkPIDController BottomShooterPID = BottomShooter.GetPIDController();
  rev::SparkRelativeEncoder TopShooterEncoder = TopShooter.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder BottomShooterEncoder = BottomShooter.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);


  // rev::CANSparkMax MagazineMotor{14, rev::CANSparkMax::MotorType::kBrushless}; //now apart of the intake 
  // frc::DigitalInput MagazineSensor{5};

  // rev::SparkMaxRelativeEncoder MagazineEncoder = MagazineMotor.GetEncoder(); //implement in intake?
  
  frc::PIDController shooterPIDController{ShooterConstants::kp, ShooterConstants::ki, 0};
  // rev::SparkMaxPIDController magPIDController = MagazineMotor.GetPIDController(); //implement in intake?

  double m_DesiredAngle = 40; 
  double testAngle = 40;

  double angleTrim = 0;
  double accumulatedError = 0;
  
  //double tempKp = 0.01;
};