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
namespace DriveConstants
{
    constexpr int kFrontLeftDriveMotorPort = 8;
    constexpr int kRearLeftDriveMotorPort = 6;
    constexpr int kFrontRightDriveMotorPort = 2;
    constexpr int kRearRightDriveMotorPort = 4;

    constexpr int kFrontLeftTurningMotorPort = 7;
    constexpr int kRearLeftTurningMotorPort = 5;
    constexpr int kFrontRightTurningMotorPort = 1;
    constexpr int kRearRightTurningMotorPort = 3;

    constexpr int kFrontLeftTurningEncoderNumber = 20; 
    constexpr int kRearLeftTurningEncoderNumber = 23;
    constexpr int kFrontRightTurningEncoderNumber = 21;
    constexpr int kRearRightTurningEncoderNumber = 22;

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

namespace ModuleConstants
{
    constexpr units::meter_t kTrackWidth = 0.58_m;  // Distance between centers of right and left wheels on robot
    constexpr units::meter_t kWheelBase = 0.58_m;  // Distance between centers of front and back wheels on robot
    constexpr units::meter_t kModuleRadius = 0.41_m;  // Distance between centers of robot and Swerve modules
    constexpr double wheelOffset = 0; //For rotation of wheels
    constexpr double gearRatio = 6.75; //L2 of gear ratio
    constexpr double kEncoderCPR = 1;
    constexpr double kWheelDiameterMeters = 0.0977;

    // Assumes the encoders are directly mounted on the wheel shafts
    constexpr double kDriveEncoderDistancePerPulse =
        (kWheelDiameterMeters * std::numbers::pi) / (kEncoderCPR) / gearRatio;

    // Assumes the encoders are directly mounted on the wheel shafts
    constexpr double kTurningEncoderDistancePerPulse =
        (std::numbers::pi * 2) / (kEncoderCPR);

    constexpr double kPModuleTurningController = 0.6;
    constexpr double kPModuleDriveController = 0.1;
    constexpr double kFFModuleDriveController = 0.259375;
}

namespace AutoConstants
{
    constexpr auto kMaxSpeed = 6.7_mps;
    constexpr auto kMaxAcceleration = 2_mps_sq;
    constexpr auto kMaxAngularSpeed = 3.142_rad_per_s * 2;
    constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq * 2;

    constexpr double kPXController = 0.1;
    constexpr double kPYController = 0.1;
    constexpr double kPThetaController = 2;
    extern const frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints;
} 

namespace DebugConstants
{
    //change this to true to debug and put most things to the smartdashboard
    constexpr bool debugShooter = true; 
    constexpr bool debugArm = false; 
    constexpr bool debugClimber = false; 
    constexpr bool debugDrive = true; 
    constexpr bool debugIntake = true; 
    constexpr bool debugLimelight = true; 
    constexpr bool debugSwerveModules = false; 
    constexpr bool debugAuto = true; 
}  

namespace ShooterConstants
{
    constexpr double AngleThreshold = 0.027;
    constexpr double RestingAngle = 20;//32; //also the low angle. TODO will change for updated offfset

    constexpr double SubwooferFrontAngle = 54;   // temp value, please test and find out
    constexpr double SubwooferSideAngle = 57;   // temp value, please test and find out
    constexpr double StageAngle = 38.18;//53.61;       // temp value, please test and find out
    constexpr double AmpAngle = 0;          // TODO find angle value
    constexpr double ShooterMaxSoftLimit = 57;
    constexpr double ShooterMinSoftLimit = 20;
    constexpr double EncoderOffSet = 0.01936; //0.03787;
    constexpr double ki = 0.0002;   //0.001
    constexpr double kp = 0.03;
    constexpr double magMotorSpeed = 0.0;
    constexpr double magKp = 0.1;
}

namespace ArmConstants
{
    constexpr double LowerInitialAngle = 0;
    constexpr double UpperInitialAngle = 0;

    constexpr double LowerFirstExtentionAngle = 30;
    constexpr double UpperFirstExtentionAngle = 0;
    
    constexpr double LowerForwardAmpExtentionAngle = 76;
    constexpr double UpperForwardAmpExtentionAngle = 138;

    constexpr double LowerBackwardAmpExtentionAngle = 50;
    constexpr double UpperBackwardAmpExtentionAngle = 206;
    
    constexpr double LowerTrapExtentionAngle = 95;
    constexpr double UpperTrapExtentionAngle = 150;

    constexpr double LowerFirstRetractionAngle = 90;
    constexpr double UpperFirstRetractionAngle = 0;

    /*TODO find angles that are actually correct. absolute encoder values are nowhere near correct right now.
        check shooter subsytem GetOffsetEncoderValue function to see how the encoders polarity is inverted with an offset added.
        if you need help talk to kaden before he leaves on wednesday 2/21 
    */
    constexpr double UpperArmSoftLimitLow = 0;
    constexpr double LowerArmSoftLimitLow = 40;
    
    constexpr double UpperArmSoftLimitHigh = 160; //change??
    constexpr double LowerArmSoftLimitHigh = 112;

    constexpr double LowerArmOffset = -49;
    constexpr double UpperArmOffset = 0.14; //TODO: test
    
    constexpr double kpLowerArm = 0.005;  //TODO: test
    constexpr double kpUpperArm = 0.0025;

    constexpr double kiLowerArm = 0.0005;  //TODO: test
    constexpr double kiUpperArm = 0.000125;

    constexpr double kiSumLowerArm = 0.0;
    constexpr double kiSumUpperArm = 0.0;
}
 