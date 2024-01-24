#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
  :
    m_frontLeft {
      kFrontLeftDriveMotorPort,
      m_EncoderType,
      kFrontLeftDriveCPR,
      kFrontLeftTurningMotorPort,
      kFrontLeftDriveEncoderReversed,
      kFrontLeftTurningEncoderNumber,
      kFrontLeftTurningEncoderReversed
    },

    m_frontRight {
      kFrontRightDriveMotorPort,
      m_EncoderType,
      kFrontRightDriveCPR,
      kFrontRightTurningMotorPort,
      kFrontRightDriveEncoderReversed,
      kFrontRightTurningEncoderNumber,
      kFrontRightTurningEncoderReversed
    },

    m_rearLeft {
      kRearLeftDriveMotorPort,
      m_EncoderType,
      kRearLeftDriveCPR,
      kRearLeftTurningMotorPort,
      kRearLeftDriveEncoderReversed,
      kRearLeftTurningEncoderNumber,
      kRearLeftTurningEncoderReversed
    },

    m_rearRight {
      kRearRightDriveMotorPort,
      m_EncoderType,
      kRearRightDriveCPR,
      kRearRightTurningMotorPort,
      kRearRightDriveEncoderReversed,
      kRearRightTurningEncoderNumber,
      kRearRightTurningEncoderReversed
    },

    m_odometry {
      kDriveKinematics,
      frc::Rotation2d(m_gyro.GetRotation2d()),
      {
        m_frontLeft.GetPosition(),
        m_frontRight.GetPosition(),
        m_rearLeft.GetPosition(),
        m_rearRight.GetPosition()
      },
      frc::Pose2d{}
    } 

    {
      // m_gyro.SetAngleAdjustment(180); //Determines the front of the robot to via gyro returns

      AutoBuilder::configureHolonomic(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ Drive(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
          PIDConstants(AutoConstants::kPXController, 0.0 , 0.0), // Translation PID constants
          PIDConstants(AutoConstants::kPThetaController, 0.0 , 0.0), // Rotation PID constants
          AutoConstants::kMaxSpeed, // Max module speed, in m/s
          ModuleConstants::kModuleRadius, // Drive base radius in meters. Distance from robot center to furthest module.
          ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        []()
        {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          auto alliance = frc::DriverStation::GetAlliance();

          if (alliance)
          {
            return alliance.value() == frc::DriverStation::Alliance::kRed;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
    }

void DriveSubsystem::Periodic()
{
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(
    frc::Rotation2d(m_gyro.GetRotation2d()),
      {
        m_frontLeft.GetPosition(),
        m_rearLeft.GetPosition(),
        m_frontRight.GetPosition(),
        m_rearRight.GetPosition()
      }
  );

  tempPose = GetPose();
  DrivePose = &tempPose;

  frc::SmartDashboard::PutNumber("DrivePosePtrX", (double)DrivePose->X());
  frc::SmartDashboard::PutNumber("DrivePosePtrRot", (double)DrivePose->Rotation().Degrees());
  frc::SmartDashboard::PutNumber("TeleRobotX", (double)GetPose().X());
  frc::SmartDashboard::PutNumber("TeleRobotY", (double)GetPose().Y());
  frc::SmartDashboard::PutNumber("TeleRobotRot", (double)GetPose().Rotation().Degrees());
}

void DriveSubsystem::Drive(
  units::meters_per_second_t xSpeed,
  units::meters_per_second_t ySpeed,
  units::radians_per_second_t rot,
  bool fieldRelative,
  bool noJoystickInput
)

{       
  if (DebugConstants::debug == true)
  {
    frc::SmartDashboard::PutNumber("ROT value: ", rot.value());
  }

  auto states = kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? 
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xSpeed,
        ySpeed, 
        rot, 
        frc::Rotation2d(m_gyro.GetRotation2d())
      ) 
    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  if (noJoystickInput == true)
  {
    fl.speed = (units::velocity::meters_per_second_t)(0);
    fr.speed = (units::velocity::meters_per_second_t)(0);
    bl.speed = (units::velocity::meters_per_second_t)(0);
    br.speed = (units::velocity::meters_per_second_t)(0);
    fl.angle = (units::angle::degree_t)(45);
    fr.angle = (units::angle::degree_t)(135);
    bl.angle = (units::angle::degree_t)(-45);
    br.angle = (units::angle::degree_t)(-135);
  } 

  if (driveSlow == true)
  {
    fl.speed = (units::velocity::meters_per_second_t)(0.5 * fl.speed);
    fr.speed = (units::velocity::meters_per_second_t)(0.5 * fr.speed);
    bl.speed = (units::velocity::meters_per_second_t)(0.5 * bl.speed);
    br.speed = (units::velocity::meters_per_second_t)(0.5 * br.speed);
  }

  if (WheelsStraight == true)
  {
    fl.angle = (units::angle::degree_t)(0);
    fr.angle = (units::angle::degree_t)(0);
    bl.angle = (units::angle::degree_t)(0);
    br.angle = (units::angle::degree_t)(0);
  }

  if (DebugConstants::debug == true){
    frc::SmartDashboard::PutNumber("Fl Desired angle",(float)fl.angle.Degrees());
    frc::SmartDashboard::PutNumber("Fr Desired angle",(float)fr.angle.Degrees());
    frc::SmartDashboard::PutNumber("Bl Desired angle",(float)bl.angle.Degrees());
    frc::SmartDashboard::PutNumber("Br Desired angle",(float)br.angle.Degrees());
    frc::SmartDashboard::PutNumber("Fl Desired speed",(float)fl.speed);
    frc::SmartDashboard::PutNumber("Fr Desired speed",(float)fr.speed);
    frc::SmartDashboard::PutNumber("Bl Desired speed",(float)bl.speed);
    frc::SmartDashboard::PutNumber("Br Desired speed",(float)br.speed);
  }

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr); 
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) 
{
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

units::degree_t DriveSubsystem::GetHeading()
{
  return m_gyro.GetRotation2d().Degrees();
}

frc2::CommandPtr DriveSubsystem::ZeroHeading()
{
  return this->RunOnce(
    [this] {
      m_gyro.Reset();
    }
  );
}

//small left right movement. Optional to have
frc2::CommandPtr DriveSubsystem::Twitch(bool direction)
{
  return this -> Run(
    [this, direction]
    {
      if (direction == true)
      {
        DriveSubsystem::Drive(0_mps, -0.35_mps, 0_rad_per_s, false, false); //right
      }
      else if (direction == false)
      {
        DriveSubsystem::Drive(0_mps, 0.35_mps, 0_rad_per_s, false, false); //left
      } 
      else
      {
        DriveSubsystem::Drive(0_mps, 0.0_mps, 0_rad_per_s, false, false); //stop
      }
    }
  );
}


double DriveSubsystem::GetTurnRate()
{
  return -m_gyro.GetRate();
}


frc::Pose2d DriveSubsystem::GetPose()
{
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
  m_odometry.ResetPosition(
    GetHeading(),
    {
      m_frontLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_rearLeft.GetPosition(),
      m_rearRight.GetPosition()
    },
    pose
  );
}

frc::Pose2d* DriveSubsystem::GetDrivePosePtr()
{
  return DrivePose;
}

frc::ChassisSpeeds DriveSubsystem::getRobotRelativeSpeeds()
{
  auto [forward, sideways, angular] = kDriveKinematics.ToChassisSpeeds(
    m_frontLeft.GetState(),
    m_frontRight.GetState(),
    m_rearLeft.GetState(),
    m_rearRight.GetState()
  );

  frc::SmartDashboard::PutNumber("Forward chassis speed", (double)forward);
  frc::SmartDashboard::PutNumber("Sideways chassis speed", (double)sideways);
  frc::SmartDashboard::PutNumber("angular chassis speed", (double)angular);

  return {forward, sideways, angular};
}

frc2::CommandPtr DriveSubsystem::FollowPathCommand(std::shared_ptr<pathplanner::PathPlannerPath> path)
{
  return FollowPathHolonomic(
    path,
    [this](){ return GetPose(); }, // Robot pose supplier
    [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    [this](frc::ChassisSpeeds speeds){ Drive(speeds.vx, speeds.vy, speeds.omega, false, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
      PIDConstants(AutoConstants::kPXController, 0.0 , 0.0), // Translation PID constants
      PIDConstants(AutoConstants::kPThetaController, 0.0 , 0.0), // Rotation PID constants
      AutoConstants::kMaxSpeed, // Max module speed, in m/s
      ModuleConstants::kModuleRadius, // Drive base radius in meters. Distance from robot center to furthest module.
      ReplanningConfig() // Default path replanning config. See the API for the options here
    ),
    []()
    {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      auto alliance = frc::DriverStation::GetAlliance();

      if (alliance)
      {
        return alliance.value() == frc::DriverStation::Alliance::kRed;
      }
        return false;
      },
      { this }// Reference to this subsystem to set requirements
  ).ToPtr();
}

DriveSubsystem::~DriveSubsystem()
{
  delete DrivePose;
}