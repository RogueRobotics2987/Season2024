// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TestMotor.h"

TestMotor::TestMotor() = default;

// This method will be called once per scheduler run
void TestMotor::Periodic() {
  // m_motor = new rev::CANSparkMax(m_motorPort, rev::CANSparkMax::MotorType::kBrushless);
  m_motor->SetOpenLoopRampRate(1);

}

frc2::CommandPtr TestMotor::Move(){
  return this->RunOnce( [this] {
    m_motor->Set(0.5);
  });
}

frc2::CommandPtr TestMotor::Stop(){
  return this->RunOnce( [this] {
    m_motor->Set(0.0);
  });
}
