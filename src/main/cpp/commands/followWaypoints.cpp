// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowWaypoints.h"

FollowWaypoints::FollowWaypoints(DriveSubsystem &drivetrain, std::vector<frc::Pose2d> waypoints, units::meters_per_second_t driveSpeed) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = &drivetrain;
  AddRequirements({m_drivetrain});
  m_waypoints.assign(waypoints.begin(), waypoints.end());
  maxSpeed = driveSpeed;

  for(int i = 0; i < (int)waypoints.size()-1; ++i){
    deltaY = fabs((double)waypoints[i].Y() - (double)waypoints[i+1].Y());
    deltaX = fabs((double)waypoints[i].X() - (double)waypoints[i+1].X());
    totalDistance = totalDistance + hypot(deltaX, deltaY);
  }
}

// Called when the command is initially scheduled.
void FollowWaypoints::Initialize() 
{
  deltaX = 0;
  deltaY = 0;
  desiredPose = m_waypoints.front();
  m_waypoints.pop_front(); 
}

// Called repeatedly when this Command is scheduled to run
void FollowWaypoints::Execute()
{
  currentPose = m_drivetrain->GetPose(); 
  
  //true if negative
  if(std::signbit((double)desiredPose.Rotation().Degrees() * (double)currentPose.Rotation().Degrees()) == true)
  {
    sign = 1;
  }
  else{
    sign = -1;
  }

  if((fabs((double)currentPose.X() - (double)desiredPose.X()) < 0.1) 
    && (fabs((double)currentPose.Y() - (double)desiredPose.Y()) < 0.1) 
    && ((fabs(DistanceBetweenAngles((double)desiredPose.Rotation().Degrees(), (double)currentPose.Rotation().Degrees()))) < 5))
  {
    if(m_waypoints.size() > 0) 
    {
      desiredPose = m_waypoints.front();
      m_waypoints.pop_front();
    }
    else
    {
      finished = true;
    }
  }

  deltaX = fabs((double)lastPose.X() - (double)currentPose.X());
  deltaY = fabs((double)lastPose.Y() - (double)currentPose.Y());
  distanceTraveled = distanceTraveled + hypot(deltaX, deltaY);

  if(DebugConstants::debug == true){
    frc::SmartDashboard::PutNumber("Hypot(X,Y)", hypot(deltaX, deltaY));
  }

  if(!finished)
  {
    // robotSpeed = 1_mps;
    if(distanceTraveled >= totalDistance){
      robotSpeed = 0.1_mps;
    }
    else if((totalDistance-distanceTraveled) <= 0.75){
      double x = (totalDistance-distanceTraveled) / 0.75;
      robotSpeed = 1 * (x) * maxSpeed; //0-100% of max speed aka Z

      if(DebugConstants::debug == true){
        frc::SmartDashboard::PutNumber("Xvalue2", x);
      }
    }
    else if(distanceTraveled <= 0.5){
      double x = distanceTraveled / 0.5;
      robotSpeed = 1 * x * maxSpeed + 0.1_mps; //0-100% of max speed aka Z

      if(DebugConstants::debug == true){
        frc::SmartDashboard::PutNumber("Xvalue1", x);
      }
    }
    else{
      robotSpeed = maxSpeed;
    }

    alpha = atan2(((double)desiredPose.Y() - (double)currentPose.Y()) , ((double)desiredPose.X() - (double)currentPose.X()));
    alpha = alpha - (double)currentPose.Rotation().Radians();
    xVal = robotSpeed * cos(alpha);
    yVal = robotSpeed * sin(alpha);

    thetaDouble = (DistanceBetweenAngles((double)desiredPose.Rotation().Degrees(), ((double)currentPose.Rotation().Degrees())));
    thetaDouble = thetaDouble * (3.14/180.0);
    thetaDouble = thetaDouble * AutoConstants::kPThetaController;

    thetaVal = thetaDouble * 1_rad_per_s;
    m_drivetrain->Drive(xVal, yVal, thetaVal, false, false);
  }

  lastPose = currentPose;
  
  if(DebugConstants::debug == true){
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
    frc::SmartDashboard::PutNumber("totalDistance", (double)totalDistance);
    frc::SmartDashboard::PutNumber("distanceTraveled", (double)distanceTraveled);
    frc::SmartDashboard::PutNumber("percentDistanceTraveled", (double)(distanceTraveled/totalDistance));
    frc::SmartDashboard::PutNumber("lastPoseX", (double)lastPose.X());
    frc::SmartDashboard::PutNumber("lastPoseY", (double)lastPose.Y());
  }
}

// Called once the command ends or is interrupted.
void FollowWaypoints::End(bool interrupted) {
  m_drivetrain->Drive(0_mps, 0_mps , 0_rad_per_s, true, false);
  distanceTraveled = 0;
  finished = false;
  totalDistance = 0;
  deltaX = 0;
  deltaY = 0;
}

// Returns true when the command should end.
bool FollowWaypoints::IsFinished()
{
  return finished;
}


double FollowWaypoints::DistanceBetweenAngles(double targetAngle, double sourceAngle){
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