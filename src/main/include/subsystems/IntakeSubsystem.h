// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>

#include "rev/CANSparkMax.h"
#include "DriveSubsystem.h"

class IntakeSubsystem : public frc2::SubsystemBase
{
  public:
    IntakeSubsystem();
    void Direction(double speed);
    void DirectionNote(double speed);
    void RunIntake(double speed);
    void StopIntake();
    bool GetIntakeFront();
    void SpitOutIntake();

    void RunMagazine(double speed);
    void StopMagazine();
    void HoldMagazine(double pos);
    double GetCurrMagEncoderVal();
    bool GetMagazineSensor();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

  private:
    double frontVal = 0.0;
    double backVal = 0.0;

    rev::CANSparkMax BackIntake{9, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax FrontIntake{10, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax CenterIntake{11, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax MagazineMotor{14, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax middleRollers{18, rev::CANSparkMax::MotorType::kBrushless};

    frc::DigitalInput intakeColorSensorFront {3};   // 0 is a place holder for the DIO port
    frc::DigitalInput MagazineSensor{5};

    rev::SparkMaxRelativeEncoder MagazineEncoder = MagazineMotor.GetEncoder(); //implement in intake?
    rev::SparkMaxPIDController magPIDController = MagazineMotor.GetPIDController(); //implement in intake?

    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};
