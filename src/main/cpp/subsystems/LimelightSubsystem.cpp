// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimelightSubsystem.h"

LimelightSubsystem::LimelightSubsystem()
{
    sleep(1);
}

// This method will be called once per scheduler run
void LimelightSubsystem::Periodic()
{
    Notetx = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx",0.0);
    Notety = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("ty",0.0);

    bluePose = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose_wpiblue", std::vector<double>(6));

    targetCount = bluePose[7];

    frc::SmartDashboard::PutNumber("targetDistance", GetDistanceFromTarget());
    frc::SmartDashboard::PutNumber("targetCount", targetCount);
}


double LimelightSubsystem::GetNotetx()
{
    return Notetx;
}

double LimelightSubsystem::GetNotety()
{
    return Notety;
}

// still need to do the hold mag seperate from the subsystem, I think in the command.
// pass in the angle trim from the shooter subsystems function get angletrim
double LimelightSubsystem::GetApriltagShooterTheta(double dist, double angleTrim)
{
    if(dist != 0.0)
    {
        frc::SmartDashboard::PutNumber("Distance AprilTag", dist);
        // return -0.2351* pow((dist+angleTrim),3) + 4.38 * pow((dist+angleTrim), 2) - 29 * (dist+angleTrim) + 89.64; //initial curve not using the PID shooter 
        // return -1.556* pow((dist+angleTrim),3) + 17.162 * pow((dist+angleTrim), 2) - 68 * (dist+angleTrim) + 127.2; //power of 3 magnolia
        // return 0.6042 * pow((dist+angleTrim),4) - 8.94 * pow((dist+angleTrim),3) + 49.7 * pow((dist+angleTrim),2) - 129.06 * (dist+angleTrim) + 168.18;
        // return -0.2238* pow((dist+angleTrim),3) + 4.1431 * pow((dist+angleTrim), 2) - 27.38 * (dist+angleTrim) + 89.093; // for 3800 rpm - unused as of now

        return 0.0487 * pow((dist+angleTrim),4) - 1.1665 * pow((dist+angleTrim),3) + 10.383 * pow((dist+angleTrim),2) - 42.875 * (dist+angleTrim) + 97.426;
    }
    else
    {
        return ShooterConstants::RestingAngle;
    }
}


double LimelightSubsystem::GetApriltagDriveMotorVal(double currentHeading, double lastHeading)
{
    if(targetCount > 0)
    {
        rz = bluePose[5]; 
        double ty = bluePose[1]; //ty

        if(bluePose[0] > (16.55/2)) //red side
        {
            double tx = 16.55 - bluePose[0]; //tx
            // double ty = bluePose[1]; //ty

            desiredHeading = atan((blueS - ty) / tx) * (180/3.14);
        }
        else if(bluePose[0] < (16.55/2)) //blue side
        {
            double tx = bluePose[0]; //tx
            // double ty = bluePose[1]; //ty

            desiredHeading = atan((blueS - ty) / tx) * (180/3.14);
            desiredHeading += 180;
        }
    }
    else
    {
        rz = rz + DistanceBetweenAngles(currentHeading, lastHeading);
    }

    // driveError = DistanceBetweenAngles(desiredHeading, currentHeading);
    driveError = DistanceBetweenAngles(desiredHeading, rz);

    frc::SmartDashboard::PutNumber("DesiredHeadingLL", desiredHeading);
    frc::SmartDashboard::PutNumber("DriveError", driveError);

    if(fabs(driveError) > 2)
    {
        return (driveError * kpApril);
    }
    else
    {
        return 0;
    }
}

double LimelightSubsystem::GetApriltagDriveError()
{
    return driveError;
}

double LimelightSubsystem::DistanceBetweenAngles(double targetAngle, double sourceAngle)
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

void LimelightSubsystem::apriltagAngleReset(double currentHeading)
{
    desiredHeading = currentHeading;
}

double LimelightSubsystem::GetDistanceFromTarget()
{
   if(targetCount > 0)
    {
        if(bluePose[0] > (16.55/2))
        {
            double tx = 16.55 - bluePose[0]; //tx
            double ty = bluePose[1]; //ty

            currentDistance = sqrt(pow(blueS - ty, 2) + pow(tx, 2));
        }
        else if(bluePose[0] < (16.55/2)) 
        {
            double tx = bluePose[0]; //tx
            double ty = bluePose[1]; //ty

            currentDistance = sqrt(pow(blueS - ty, 2) + pow(tx, 2));
        }

        lastDistance = currentDistance;
        return currentDistance;
    }
    else
    {
        return lastDistance;
    }
}

int LimelightSubsystem::GetNumTargets()
{
    return targetCount;
}