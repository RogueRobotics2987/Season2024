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


class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();
  double GetOffSetEncoderValueLower();
  double GetOffSetEncoderValueUpper();
  
  //DO NOT SET TO ZERO doesnt acutally update....
  void setLowerArmAngle(double desiredAngle);

//  void setVoltage(double speed);
  void accumulateErrorLower();

  void RunLowerArm();

  void ZeroIntergral();

  void StopWheels();

  void runArmWheels(double speed);
  void stopArmWheels();
  void FollowShooter(double error);

  void MoveLowerArm();
  double getLowerArmError();
  double getLowerEncoderPos();  
  double DistanceBetweenAngles(double targetAngle, double sourceAngle);

  bool compareHasNote(bool other);


  // void defaultArmPos();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::CANSparkMax LowerArm{17, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax ArmWheels{19, rev::CANSparkMax::MotorType::kBrushless};

  // rev::SparkMaxAlternateEncoder m_LowerArmEncoder{LowerArm.GetAlternateEncoder(125)}; //8192?
  frc::DutyCycleEncoder m_LowerArmEncoder{7}; //lower encoder value degrees = 30, lower encode valuse degrees extended = 112
  double LoweraccumulatedError = 0;
  double m_LowerDesired = 0;

  // enum armState {INITIAL, LOWER_ARM_EXTEND_INITIAL, UPPER_ARM_EXTEND_INITIAL, ARM_FINAL, DROP, ARM_RETRACT_INITIAL, ARM_RETRACT_FINAL};
  // armState state = INITIAL;
  // frc::XboxController* m_Xbox = nullptr;
  // bool HasNote = false;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.


};
