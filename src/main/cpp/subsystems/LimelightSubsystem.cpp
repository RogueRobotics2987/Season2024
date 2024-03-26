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
    targetIDs.clear();

    Notetx = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx",0.0);
    Notety = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("ty",0.0);

    // Notetx = LimelightHelpers::getTX("limelight-back");
    // Notety = LimelightHelpers::getTY("limelight-back");

    // targetID = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tid", -1);


    bluePose = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose_wpiblue", std::vector<double>(6));

    targetCount = bluePose[7];

    frc::SmartDashboard::PutNumber("targetDistance", GetDistanceFromTarget());
    frc::SmartDashboard::PutNumber("targetCount", targetCount);

    // auto threeDPose = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose",std::vector<double>(6));
    // auto threeDPose2 = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose blue",std::vector<double>(6));

    // double distanceToTagInMeters = threeDPose[0];
    // frc::SmartDashboard::PutNumber("Distance", distanceToTagInMeters);
    // frc::SmartDashboard::PutNumber("Distance2", threeDPose2[0]);

    // result = camera.GetLatestResult();
    // hasTarget = result.HasTargets();

    // filteredTargetID = -1;

    // if(hasTarget == true)
    // {
    //     tempTargets = result.GetTargets();
    //     myTargets.assign(tempTargets.begin(), tempTargets.end());

    //     for(unsigned int i = 0; i < myTargets.size(); i++)
    //     {
    //         targetIDs.emplace_back(myTargets.at(i).GetFiducialId()); //TODO remove(?)

    //         if(myTargets.at(i).GetFiducialId() == 4 || myTargets.at(i).GetFiducialId() == 7)
    //         {
    //             filteredTarget = myTargets.at(i);
    //             filteredTargetID = filteredTarget.GetFiducialId();
    //             //TODO have the yaw on our robot search for 0. include our specific id into the calc dist to target

    //             filteredRange = photon::PhotonUtils::CalculateDistanceToTarget(
    //             CAMERA_HEIGHT, TAREGT_HEIGHT, CAMERA_PITCH,
    //             units::degree_t{filteredTarget.GetPitch()});
    //         }
    //     }   
    // }

    // if(DebugConstants::debugLimelight == true)
    // {
    //     frc::SmartDashboard::PutNumber("FilteredRange", filteredRange.value());
    //     frc::SmartDashboard::PutNumber("FilteredYaw", filteredTarget.GetYaw());
    //     frc::SmartDashboard::PutNumber("FilteredPitch", filteredTarget.GetPitch());
    //     frc::SmartDashboard::PutNumber("FilteredID", filteredTargetID);
    // }
}

double LimelightSubsystem::GetAmptx()
{
    if(filteredTargetID == 5)
    {
        return PhotonYaw();
    } 
    else 
    {
        return 0;
    }
}

bool LimelightSubsystem::PhotonHasTarget()
{
    bool hasTargetFiltered = false;

    if(hasTarget == true)
    {
        if(filteredTargetID == 4 || filteredTargetID == 7)
        {
            hasTargetFiltered = true;
        }
    }

    return hasTargetFiltered;
}

double LimelightSubsystem::PhotonYaw()
{
    double yaw = filteredTarget.GetYaw();
    return yaw;
}

double LimelightSubsystem::FilteredPhotonYaw()
{
    double staticOffset = 2;

    return PhotonYawMap(filteredTarget.GetYaw()) + staticOffset;
}

double LimelightSubsystem::PhotonYawMap(double originalVal)
{
    double convertedVal = 0.0;
    double expoOffsetVal = 0.0;
    double halfFOV = 31.66;
    double A = 5 / (halfFOV * halfFOV);     // 5 is max error at edge of FOV

    expoOffsetVal = A * (originalVal * originalVal);

    if(originalVal >= 0)
    {
        convertedVal = originalVal - expoOffsetVal;
    }
    else if(originalVal < 0)
    {
        convertedVal = originalVal + expoOffsetVal;
    }

    return convertedVal;
}

double LimelightSubsystem::GetNotetx()
{
    return Notetx;
}

double LimelightSubsystem::GetNotety()
{
    return Notety;
}

double LimelightSubsystem::FilteredDistance()
{
    return filteredRange.value();
}

photon::PhotonTrackedTarget LimelightSubsystem::GetFilteredTarget()
{
    return filteredTarget;
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
        return 0.6042 * pow((dist+angleTrim),4) - 8.94 * pow((dist+angleTrim),3) + 49.7 * pow((dist+angleTrim),2) - 129.06 * (dist+angleTrim) + 168.18;
        // return -0.2238* pow((dist+angleTrim),3) + 4.1431 * pow((dist+angleTrim), 2) - 27.38 * (dist+angleTrim) + 89.093; // for 3800 rpm - unused as of now
    }
    else
    {
        return ShooterConstants::RestingAngle;
    }
}

// double LimelightSubsystem::GetApriltagDriveMotorVal(double currentHeading)
// {
//     if(filteredTargetID == 4 || filteredTargetID == 7)
//     {
//         txApril = FilteredPhotonYaw();
//         desiredHeading = currentHeading + -txApril;// calculated actual angle instead of the error
//     }

//     driveError = DistanceBetweenAngles(desiredHeading, currentHeading);

//     // std::cout << "txApril " << txApril << std::endl; 
//     // std::cout << "driveError " << driveError << std::endl;
//     // std::cout << "desiredHeading " << desiredHeading << std::endl; 
//     // std::cout << "currentHeading " << currentHeading << std::endl; 

//     return (driveError * kpApril);
// }

double LimelightSubsystem::GetApriltagDriveMotorVal(double currentHeading)
{
    if(targetCount > 0)
    {
        if(bluePose[0] > (16.55/2)) //red side
        {
            double tx = 16.55 - bluePose[0]; //tx
            double ty = bluePose[1]; //ty

            desiredHeading = atan((blueS - ty) / tx) * (180/3.14);
        }
        else if(bluePose[0] < (16.55/2)) //blue side
        {
            double tx = bluePose[0]; //tx
            double ty = bluePose[1]; //ty

            desiredHeading = atan((blueS - ty) / tx) * (180/3.14);
            desiredHeading += 180;
        }
    }

    driveError = DistanceBetweenAngles(desiredHeading, currentHeading);

    if(fabs(driveError) > 2.0)
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

            return sqrt(pow(blueS - ty, 2) + pow(tx, 2));
        }
        else if(bluePose[0] < (16.55/2)) 
        {
            double tx = bluePose[0]; //tx
            double ty = bluePose[1]; //ty

            return sqrt(pow(blueS - ty, 2) + pow(tx, 2));
        }
    }
    else
    {
        return 0;
    }
}