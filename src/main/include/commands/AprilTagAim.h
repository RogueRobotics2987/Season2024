// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "networktables/NetworkTableInstance.inc"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LimelightSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AprilTagAim
    : public frc2::CommandHelper<frc2::Command, AprilTagAim> 
{
  public:
    AprilTagAim(); 
    AprilTagAim(LimelightSubsystem &limelight, DriveSubsystem &drivetrain, frc::XboxController &driveXbox, ShooterSubsystem &shooter, frc::XboxController &auxController);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
    
    float Deadzone(float x);

    double kp = 0.02206;
    double speedX = 0;
    double speedY = 0;
    bool NoJoystickInput = false;

  private:
    LimelightSubsystem* m_limelight = nullptr;
    DriveSubsystem* m_drivetrain = nullptr;
    ShooterSubsystem* m_shooter = nullptr;
    frc::XboxController* m_driverController = nullptr;
    frc::XboxController* m_auxController = nullptr;
    
    units::angular_velocity::radians_per_second_t rotApril = units::angular_velocity::radians_per_second_t(0);
    
    double currentHeading = 0;
    double desiredHeading = 0;
    int filteredTargetID = -1;
    double error = 0;
    bool finished = false;
    bool shoot = false;
    int time = 0;
};