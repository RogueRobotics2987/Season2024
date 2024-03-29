// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowWaypoints.h"

FollowWaypoints::FollowWaypoints(
  DriveSubsystem &drivetrain,
  LimelightSubsystem &limePose,
  std::vector<frc::Pose2d> waypoints,
  std::vector<units::meters_per_second_t> driveSpeed,
  std::vector<units::meters_per_second_t> cruiseSpeed,
  bool limeLight) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = &drivetrain;
  m_limePose = &limePose;
  AddRequirements({m_drivetrain});
  AddRequirements({m_limePose});

  m_waypoints = waypoints;

  // m_waypoints.assign(waypoints.begin(), waypoints.end());
  m_driveSpeed.assign(driveSpeed.begin(), driveSpeed.end());
  m_cruiseSpeed.assign(cruiseSpeed.begin(), cruiseSpeed.end());

  limeBool = limeLight;
}

// Called when the command is initially scheduled.
void FollowWaypoints::Initialize() 
{
  std::cout << m_waypoints.size() << std::endl;

  if(!((fabs((double)m_waypoints[0].X() - (double)m_drivetrain->GetPose().X()) < threshold) && (fabs((double)m_waypoints[0].Y() - (double)m_drivetrain->GetPose().Y()) < threshold)))
  {
    m_waypoints.emplace(m_waypoints.begin(), m_drivetrain->GetPose());
  }

  for(int i = 0; i < (int)m_waypoints.size()-1; ++i)
  {
    deltaY = fabs((double)m_waypoints[i].Y() - (double)m_waypoints[i+1].Y());
    deltaX = fabs((double)m_waypoints[i].X() - (double)m_waypoints[i+1].X());
    m_waypointDistance.push_back(hypot(deltaX, deltaY));
  }

  deltaX = 0;
  deltaY = 0;
  lastPointSpeed = 0_mps;
  
  desiredPose = m_waypoints.front();
  m_waypoints.erase(m_waypoints.begin()); 
  pointSpeed = m_driveSpeed.front();
  m_driveSpeed.pop_front();
  cruiseSpeed = m_cruiseSpeed.front();
  m_cruiseSpeed.pop_front();
  currentDistance = 0;

  lastPose = m_drivetrain->GetPose();

  accumulatedError = 0;
}

// Called repeatedly when this Command is scheduled to run
void FollowWaypoints::Execute()
{
  currentPose = m_drivetrain->GetPose();

  if((fabs((double)currentPose.X() - (double)desiredPose.X()) < threshold) 
    && (fabs((double)currentPose.Y() - (double)desiredPose.Y()) < threshold))
  {
    if(m_waypoints.size() > 0) 
    {
      desiredPose = m_waypoints.front();
      m_waypoints.erase(m_waypoints.begin()); 
      lastPointSpeed = pointSpeed;
      pointSpeed = m_driveSpeed.front();
      m_driveSpeed.pop_front();
      cruiseSpeed = m_cruiseSpeed.front();
      m_cruiseSpeed.pop_front();
      currentDistance = m_waypointDistance.front();
      m_waypointDistance.pop_front();
      distanceTraveled = 0;

      if(m_waypoints.size() == 0)
      {
        threshold = 0.05;
      }
    }
    else
    {
      finished = true;
    }
  }

  deltaX = fabs((double)lastPose.X() - (double)currentPose.X());
  deltaY = fabs((double)lastPose.Y() - (double)currentPose.Y());
  distanceTraveled = distanceTraveled + hypot(deltaX, deltaY);

  if(DebugConstants::debugAuto == true)
  {
    frc::SmartDashboard::PutNumber("Hypot(X,Y)", hypot(deltaX, deltaY));
  }

  if(!finished)
  {
    if((distanceTraveled) >= (currentDistance * 1.1))
    {
      robotSpeed = 1_mps;
      frc::SmartDashboard::PutString("AutoSpeed", "OVERSHOOT");
    }
    else if(fabs(currentDistance-distanceTraveled) <= 0.2)
    {
      accumulatedError += 5E-3 * (currentDistance - distanceTraveled);
      double x = (currentDistance-distanceTraveled) / 0.2;
      robotSpeed = 1 * x * (cruiseSpeed - pointSpeed) + pointSpeed + (units::meters_per_second_t)accumulatedError; //0-100% of max speed aka Z

      frc::SmartDashboard::PutString("AutoSpeed", "SLOWDOWN");

      if(DebugConstants::debugAuto == true)
      {
        frc::SmartDashboard::PutNumber("Xvalue2", x);
      }
    }
    else
    {
      robotSpeed = cruiseSpeed;
      frc::SmartDashboard::PutString("AutoSpeed", "CruiseSpeed");
    }

    alpha = atan2(((double)desiredPose.Y() - (double)currentPose.Y()) , ((double)desiredPose.X() - (double)currentPose.X()));
    alpha = alpha - (double)currentPose.Rotation().Radians();
    xVal = robotSpeed * cos(alpha);
    yVal = robotSpeed * sin(alpha);

    thetaDouble = (DistanceBetweenAngles((double)desiredPose.Rotation().Degrees(), ((double)currentPose.Rotation().Degrees())));
    thetaDouble = thetaDouble * (3.14/180.0);
    thetaDouble = thetaDouble * AutoConstants::kPThetaController;

    thetaVal = thetaDouble * 1_rad_per_s;
    m_drivetrain->Drive(xVal, yVal, false, false);

    if(limeBool == true)
    {
      units::angular_velocity::radians_per_second_t rotApril = 
        units::angular_velocity::radians_per_second_t(m_limePose->GetApriltagDriveMotorVal(currentPose.Rotation().Degrees().value(), lastPose.Rotation().Degrees().value()));

      m_drivetrain->Drive(rotApril, false, false);
    }
    else
    {
      m_drivetrain->Drive(thetaVal, false, false);
    }
  }

  lastPose = currentPose;
  
  if(DebugConstants::debugAuto == true)
  {
    frc::SmartDashboard::PutNumber("DesiredPoseX", (double)desiredPose.X());
    frc::SmartDashboard::PutNumber("DesiredPoseY", (double)desiredPose.Y());
    frc::SmartDashboard::PutNumber("DesiredAngle", (double)desiredPose.Rotation().Degrees());
    frc::SmartDashboard::PutNumber("CurrentPoseX", (double)currentPose.X());
    frc::SmartDashboard::PutNumber("CurrentPoseY", (double)currentPose.Y());
    frc::SmartDashboard::PutNumber("CurrentAngle", (double)currentPose.Rotation().Degrees());
    frc::SmartDashboard::PutNumber("AutoDriveAlpha", (double)alpha);
    frc::SmartDashboard::PutNumber("AutoDriveYval", (double)yVal);
    frc::SmartDashboard::PutNumber("AutoDriveXval", (double)xVal);
    frc::SmartDashboard::PutNumber("robotSpeed", (double)robotSpeed);
    frc::SmartDashboard::PutNumber("currentDistance", (double)currentDistance);
    frc::SmartDashboard::PutNumber("distanceTraveled", (double)distanceTraveled);
    frc::SmartDashboard::PutNumber("percentDistanceTraveled", (double)(distanceTraveled/currentDistance));
    frc::SmartDashboard::PutNumber("lastPoseX", (double)lastPose.X());
    frc::SmartDashboard::PutNumber("lastPoseY", (double)lastPose.Y());
    frc::SmartDashboard::PutNumber("Threshold", threshold);
    frc::SmartDashboard::PutNumber("accumulatedError", accumulatedError);
    frc::SmartDashboard::PutNumber("cruiseSpeed", (double)cruiseSpeed);
    frc::SmartDashboard::PutNumber("pointSpeed", (double)pointSpeed);
  }
}

// Called once the command ends or is interrupted.
void FollowWaypoints::End(bool interrupted)
{
  m_drivetrain->Drive(0_mps, 0_mps , 0_rad_per_s, false, false);
  distanceTraveled = 0;
  finished = false;
  totalDistance = 0;
  deltaX = 0;
  deltaY = 0;
  threshold = 0.1;
}

// Returns true when the command should end.
bool FollowWaypoints::IsFinished()
{
  return finished;
}

double FollowWaypoints::DistanceBetweenAngles(double targetAngle, double sourceAngle)
{
  double a = targetAngle - sourceAngle;
  if(a > 180)
  {
    a = a + -360;
  }
  else if(a < -180)
  {
    a = a + 360;
  }
  else
  {
    a = a;
  }

  return a;
}