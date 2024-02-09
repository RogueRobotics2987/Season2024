#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"

class SwerveModuleSubsystem : public frc2::SubsystemBase
{
  public:
    SwerveModuleSubsystem();
    SwerveModuleSubsystem(
      int m_MotorController,
      rev::SparkRelativeEncoder::Type m_EncoderType,
      int m_counts_per_rev,
      int m_MotorControllerTurning,
      bool driveEncoderReversed,
      int TurningEncoderNumber,
      bool turningEncoderReversed
    );

    ~SwerveModuleSubsystem();
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();
    void SetDesiredState(const frc::SwerveModuleState& state);
    void ResetEncoders();
    void ConfigMotorControllers();
    frc::ProfiledPIDController<units::radians> GetTurnPID();
 
  private:
    // We have to use meters here instead of radians due to the fact that
    // ProfiledPIDController's constraints only take in meters per second and
    // meters per second squared.
    static constexpr auto kModuleMaxAngularVelocity =
        units::radians_per_second_t{std::numbers::pi * 8.0};
    static constexpr auto kModuleMaxAngularAcceleration =
        units::radians_per_second_squared_t{std::numbers::pi * 16.0};

    rev::CANSparkMax* m_driveMotor;
    rev::CANSparkMax* m_turningMotor;
    
    rev::SparkRelativeEncoder* m_driveEncoder;
    rev::SparkRelativeEncoder::Type m_EncoderType;
    int m_counts_per_rev;
    ctre::phoenix::sensors::CANCoder* m_turningEncoder;

    bool m_reverseDriveEncoder;
    bool m_reverseTurningEncoder;

    frc::ProfiledPIDController<units::radians> m_turningPIDController{
      ModuleConstants::kPModuleTurningController,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}
    };
    frc::PIDController m_drivePIDController{
      ModuleConstants::kPModuleDriveController, 0, 0
    };
};
