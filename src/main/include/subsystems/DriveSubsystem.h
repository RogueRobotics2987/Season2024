#pragma once
#include <AHRS.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <frc2/command/CommandPtr.h>
#include <iostream>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/commands/FollowPathHolonomic.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/commands/FollowPathCommand.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/util/PIDConstants.h>

#include "Constants.h"
#include "SwerveModuleSubsystem.h"

using namespace pathplanner;

class DriveSubsystem : public frc2::SubsystemBase 
{
  public:
    DriveSubsystem();
    ~DriveSubsystem();
    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    // Subsystem methods go here.

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
     * and the linear speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to
     *                      the field.
     * @param noJoystickinput if there is no joystick input, the wheels will go to the 45 degree (X) position
     */
    void Drive(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
              bool fieldRelative, bool noJoystickInput);

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    void ResetEncoders();

    /**
     * Sets the drive MotorControllers to a power from -1 to 1.
     */
    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    units::degree_t GetHeading();
    float GetPitch();
    float GetRoll();
    float GetRawGyroX();
    frc2::CommandPtr ZeroHeading();
    frc2::CommandPtr FieldOrientatedTrue(); //field orientated driving
    frc2::CommandPtr FieldOrientatedFalse(); //robot centric driving
    frc2::CommandPtr SetAngleAdjustment(double angle);
    
    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    double GetTurnRate();

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    frc::Pose2d GetPose();

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    void ResetOdometry(frc::Pose2d pose);
    frc2::CommandPtr SetDriveSlow(bool m_bool);
    void ConfigMotorControllers();
    frc2::CommandPtr Twitch(bool direction);

    frc::SwerveDriveKinematics<4> kDriveKinematics
    {
      frc::Translation2d{ModuleConstants::kWheelBase / 2, ModuleConstants::kTrackWidth / 2},
      frc::Translation2d{ModuleConstants::kWheelBase / 2, -ModuleConstants::kTrackWidth / 2},
      frc::Translation2d{-ModuleConstants::kWheelBase / 2, ModuleConstants::kTrackWidth / 2},
      frc::Translation2d{-ModuleConstants::kWheelBase / 2, -ModuleConstants::kTrackWidth / 2}
    };

    frc::Pose2d* GetDrivePosePtr();
    frc::ChassisSpeeds getRobotRelativeSpeeds();
    frc2::CommandPtr FollowPathCommand(std::shared_ptr<pathplanner::PathPlannerPath> path);

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    SwerveModuleSubsystem m_frontLeft;
    SwerveModuleSubsystem m_frontRight;
    SwerveModuleSubsystem m_rearLeft;
    SwerveModuleSubsystem m_rearRight;

    // The gyro sensor
    AHRS m_gyro{frc::SerialPort::kMXP};

    // Odometry class for tracking robot pose
    // 4 defines the number of modules
    frc::SwerveDriveOdometry<4> m_odometry;
    bool driveSlow = false;
    bool WheelsStraight = false;

    frc::Pose2d* DrivePose;
    frc::Pose2d tempPose;

    frc::Field2d m_field;

    int numAT = 0;
    bool fieldOrientated = false;
};
