#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <numbers>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "rev/SparkRelativeEncoder.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace DriveConstants {
    constexpr int kFrontLeftDriveMotorPort = 42;
    constexpr int kRearLeftDriveMotorPort = 18;
    constexpr int kFrontRightDriveMotorPort = 56;
    constexpr int kRearRightDriveMotorPort = 46;

    constexpr int kFrontLeftTurningMotorPort = 48;
    constexpr int kRearLeftTurningMotorPort = 43;
    constexpr int kFrontRightTurningMotorPort = 9;
    constexpr int kRearRightTurningMotorPort = 47;

    constexpr int kFrontLeftTurningEncoderNumber = 13; 
    constexpr int kRearLeftTurningEncoderNumber = 16;
    constexpr int kFrontRightTurningEncoderNumber = 14;
    constexpr int kRearRightTurningEncoderNumber = 15;

    constexpr bool kFrontLeftTurningEncoderReversed = false;
    constexpr bool kRearLeftTurningEncoderReversed = false;
    constexpr bool kFrontRightTurningEncoderReversed = false;
    constexpr bool kRearRightTurningEncoderReversed = false;

    constexpr bool kFrontLeftDriveEncoderReversed = true;
    constexpr bool kRearLeftDriveEncoderReversed = false;
    constexpr bool kFrontRightDriveEncoderReversed = true;
    constexpr bool kRearRightDriveEncoderReversed = false;

    constexpr int kFrontLeftDriveCPR = 42;
    constexpr int kRearLeftDriveCPR = 42;
    constexpr int kFrontRightDriveCPR = 42;
    constexpr int kRearRightDriveCPR = 42;

    constexpr int kFrontLeftTurningCPR = 1;
    constexpr int kRearLeftTurningCPR = 1;
    constexpr int kFrontRightTurningCPR = 1;
    constexpr int kRearRightTurningCPR = 1;

    constexpr rev::SparkRelativeEncoder::Type m_EncoderType = rev::SparkRelativeEncoder::Type::kHallSensor;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive. The RobotPy Characterization
    // Toolsuite provides a convenient tool for obtaining these values for your
    // robot.
    constexpr auto ks = 1_V;
    constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

    // Example value only - as above, this must be tuned for your drive!
    constexpr double kPFrontLeftVel = 0.5;
    constexpr double kPRearLeftVel = 0.5;
    constexpr double kPFrontRightVel = 0.5;
    constexpr double kPRearRightVel = 0.5;
}  

namespace ModuleConstants {
    constexpr units::meter_t kTrackWidth = 0.58_m;  // Distance between centers of right and left wheels on robot
    constexpr units::meter_t kWheelBase = 0.58_m;  // Distance between centers of front and back wheels on robot
    constexpr units::meter_t kModuleRadius = 0.41_m;  // Distance between centers of robot and Swerve modules
    constexpr double wheelOffset = 0; //For rotation of wheels
    constexpr double gearRatio = 6.75; //L2 of gear ratio
    constexpr double kEncoderCPR = 1;
    constexpr double kWheelDiameterMeters = 0.0977;

    constexpr double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * std::numbers::pi) / (kEncoderCPR) / gearRatio;

    constexpr double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (std::numbers::pi * 2) / (kEncoderCPR);

    constexpr double kPModuleTurningController = 0.6;
    constexpr double kPModuleDriveController = 0.1;
    constexpr double kFFModuleDriveController = 0.259375;
}

namespace AutoConstants {
    constexpr auto kMaxSpeed = 1.0_mps;
    constexpr auto kMaxAcceleration = 2_mps_sq;
    constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
    constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

    constexpr double kPXController = 0.1;
    constexpr double kPYController = 0.1;
    constexpr double kPThetaController = 1;
    extern const frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints;
} 

namespace DebugConstants {
    constexpr bool debug = false; //change this to true to debug and put most things to the smartdashboard
}  