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
      frc::Rotation2d(m_gyro.GetRotation2d().Radians()),
      {
        m_frontLeft.GetPosition(),
        m_frontRight.GetPosition(),
        m_rearLeft.GetPosition(),
        m_rearRight.GetPosition()
      },
      frc::Pose2d{}
    }
    {
      sleep(1);
    }

void DriveSubsystem::Periodic(){
  // Implementation of subsystem periodic method goes here.

  frc::SmartDashboard::PutNumber("NavX temp", m_gyro.GetTempC());
  frc::SmartDashboard::PutNumber("OrientationOffset", orientationOffset.Degrees().value());
  //frc::SmartDashboard::PutNumber("heading", GetHeading());

  m_odometry.Update(
    frc::Rotation2d(frc::Rotation2d(m_gyro.GetRotation2d().Radians())),
      {
        m_frontLeft.GetPosition(),
        m_frontRight.GetPosition(),
        m_rearLeft.GetPosition(),
        m_rearRight.GetPosition()
      }
  );

  tempPose = GetPose();
  DrivePose = &tempPose;

  periodicHelper();

  frc::SmartDashboard::PutNumber("Rotation", GetPose().Rotation().Degrees().value());

  if(DebugConstants::debugDrive == true)
  {
    frc::SmartDashboard::PutNumber("DrivePosePtrX", (double)DrivePose->X());
    frc::SmartDashboard::PutNumber("DrivePosePtrRot", (double)DrivePose->Rotation().Degrees());
    frc::SmartDashboard::PutBoolean("GyroConnection",  m_gyro.IsConnected());
    frc::SmartDashboard::PutBoolean("GyroCalibrating",  m_gyro.IsCalibrating());
    frc::SmartDashboard::PutString("GyroFirmware", m_gyro.GetFirmwareVersion());
  }
}

void DriveSubsystem::Drive(
  units::meters_per_second_t xDriveSpeed,
  units::meters_per_second_t yDriveSpeed,
  units::radians_per_second_t rotDrive,
  bool driveFieldRelative,
  bool driveNoJoystickInput)
{
  //set local member variables for X,Y,Rot, field relative, noJoystick
  xSpeed = xDriveSpeed;
  ySpeed = yDriveSpeed;
  rot = rotDrive;
  fieldRelative = driveFieldRelative;
  noJoystickInput = driveNoJoystickInput;
}

void DriveSubsystem::Drive(
  units::meters_per_second_t xDriveSpeed,
  units::meters_per_second_t yDriveSpeed,
  bool driveFieldRelative,
  bool driveNoJoystickInput)
{
  xSpeed = xDriveSpeed;
  ySpeed = yDriveSpeed;
  fieldRelative = driveFieldRelative;
  noJoystickInput = driveNoJoystickInput;
}

void DriveSubsystem::Drive(
  units::radians_per_second_t rotDrive,
  bool driveFieldRelative,
  bool driveNoJoystickInput)
{
  rot = rotDrive;
  fieldRelative = driveFieldRelative;
  noJoystickInput = driveNoJoystickInput;
}

void DriveSubsystem::periodicHelper()
{
  if (DebugConstants::debugDrive == true)
  {
    frc::SmartDashboard::PutNumber("ROT value: ", rot.value());
  }

  auto states = kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? 
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xSpeed,
        ySpeed, 
        rot, 
        frc::Rotation2d(m_gyro.GetRotation2d().Radians() - orientationOffset.Radians())
      ) 
    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  double speedFL = m_frontLeft.GetState().speed.value();
  double speedFR = m_frontRight.GetState().speed.value();
  double speedBL = m_rearLeft.GetState().speed.value();
  double speedBR = m_rearRight.GetState().speed.value();

  double totalVelocity = speedFL + speedFR + speedBL + speedBR;

  if(totalVelocity < 0.1)
  {
    float errorFL = (float)m_frontLeft.GetTurnPID().GetPositionError();
    float errorFR = (float)m_frontRight.GetTurnPID().GetPositionError();
    float errorBL = (float)m_rearLeft.GetTurnPID().GetPositionError();
    float errorBR = (float)m_rearLeft.GetTurnPID().GetPositionError();

    float totalError = fabs(errorFL) + fabs(errorFR) + fabs(errorBL) + fabs(errorBR);
    
    if(totalError > 0.21)
    {
      fl.speed = (units::velocity::meters_per_second_t)(0);
      fr.speed = (units::velocity::meters_per_second_t)(0);
      bl.speed = (units::velocity::meters_per_second_t)(0);
      br.speed = (units::velocity::meters_per_second_t)(0);
    }
  }

  if (noJoystickInput == true)
  {
    fl.speed = (units::velocity::meters_per_second_t)(0);
    fr.speed = (units::velocity::meters_per_second_t)(0);
    bl.speed = (units::velocity::meters_per_second_t)(0);
    br.speed = (units::velocity::meters_per_second_t)(0);
    fl.angle = (units::angle::degree_t)(45);
    fr.angle = (units::angle::degree_t)(-45);
    bl.angle = (units::angle::degree_t)(-45);
    br.angle = (units::angle::degree_t)(45);
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


  if (DebugConstants::debugDrive == true)
  {
    frc::SmartDashboard::PutNumber("Fl Desired angle",(float)fl.angle.Degrees());
    frc::SmartDashboard::PutNumber("Fr Desired angle",(float)fr.angle.Degrees());
    frc::SmartDashboard::PutNumber("Bl Desired angle",(float)bl.angle.Degrees());
    frc::SmartDashboard::PutNumber("Br Desired angle",(float)br.angle.Degrees());
    frc::SmartDashboard::PutNumber("Fl Desired speed",(float)fl.speed);
    frc::SmartDashboard::PutNumber("Fr Desired speed",(float)fr.speed);
    frc::SmartDashboard::PutNumber("Bl Desired speed",(float)bl.speed);
    frc::SmartDashboard::PutNumber("Br Desired speed",(float)br.speed);
    frc::SmartDashboard::PutNumber("TeleRobotX", (double)GetPose().X());
    frc::SmartDashboard::PutNumber("TeleRobotY", (double)GetPose().Y());
    frc::SmartDashboard::PutNumber("TeleRobotRot", (double)GetPose().Rotation().Degrees());
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
  return m_gyro.GetRotation2d().Degrees() - orientationOffset.Degrees();
}

void DriveSubsystem::ZeroHeading(frc::Rotation2d startRot)
{
  orientationOffset = (startRot.Degrees());
  m_gyro.Reset();
}

frc2::CommandPtr DriveSubsystem::ZeroHeadingCmd()
{
  return this->RunOnce(
    [this] {
      orientationOffset = frc::Rotation2d(0_rad);
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

  if(DebugConstants::debugDrive == true)
  {
    frc::SmartDashboard::PutNumber("Forward chassis speed", (double)forward);
    frc::SmartDashboard::PutNumber("Sideways chassis speed", (double)sideways);
    frc::SmartDashboard::PutNumber("angular chassis speed", (double)angular);
  }

  return {forward, sideways, angular};
}

void DriveSubsystem::InitSendable(wpi::SendableBuilder& builder)
{
  builder.SetSmartDashboardType("SwerveDrive");

  builder.AddDoubleProperty("Front Left Angle", [this] { return m_frontLeft.GetState().angle.Radians().value(); }, nullptr);
  builder.AddDoubleProperty("Front Left Velocity", [this] { return (double)m_frontLeft.GetState().speed; }, nullptr);

  builder.AddDoubleProperty("Front Right Angle", [this] { return m_frontRight.GetState().angle.Radians().value(); }, nullptr);
  builder.AddDoubleProperty("Front Right Velocity", [this] { return (double)m_frontRight.GetState().speed; }, nullptr);

  builder.AddDoubleProperty("Back Left Angle", [this] { return m_rearLeft.GetState().angle.Radians().value(); }, nullptr);
  builder.AddDoubleProperty("Back Left Velocity", [this] { return (double)m_rearLeft.GetState().speed; }, nullptr);

  builder.AddDoubleProperty("Back Right Angle", [this] { return m_rearRight.GetState().angle.Radians().value(); }, nullptr);
  builder.AddDoubleProperty("Back Right Velocity", [this] { return (double)m_rearRight.GetState().speed; }, nullptr);

  builder.AddDoubleProperty("Robot Angle", [this] { return GetPose().Rotation().Radians().value(); }, nullptr);
}

DriveSubsystem::~DriveSubsystem()
{
  delete DrivePose;
}