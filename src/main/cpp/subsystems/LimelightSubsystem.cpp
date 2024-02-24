// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimelightSubsystem.h"

LimelightSubsystem::LimelightSubsystem() = default;

// This method will be called once per scheduler run
void LimelightSubsystem::Periodic()
{
    targetIDs.clear();
    
    //works to read values from the network tables for limelight and pushes them back out to confirm we are reading values
    // botPose = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose", std::span<const double>({1 , 0, 0, 0, 0, 0}));

    //TODO Coment this back in eventually once limelights are put onto the robot

    //AprilTagstx = ;//nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tx",0.0);
    //AprilTagsty = ;//nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("ty",0.0);
    Notetx = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx",0.0);
    Notety = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("ty",0.0);
    auto threeDPose = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose",std::vector<double>(6));
    auto threeDPose2 = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose blue",std::vector<double>(6));

    //std::cout << threeDPose.alue << std::endl;

    double distanceToTagInMeters = threeDPose[0];
    frc::SmartDashboard::PutNumber("Distance", distanceToTagInMeters);
    frc::SmartDashboard::PutNumber("Distance2", threeDPose2[0]);

    std::span<const photon::PhotonTrackedTarget> tempTargets = result.GetTargets();
    myTargets.assign(tempTargets.begin(), tempTargets.end());

    for(unsigned int i = 0; i < myTargets.size(); i++)
    {
      targetIDs.emplace_back(myTargets.at(i).GetFiducialId());

      if(myTargets.at(i).GetFiducialId() == 4 || myTargets.at(i).GetFiducialId() == 7)
      {
        filteredTarget = myTargets.at(i);
        filteredTargetID = filteredTarget.GetFiducialId();
        //TODO have the yaw on our robot search for 0. include our specific id into the calc dist to target

        filteredRange = photon::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT, TAREGT_HEIGHT, CAMERA_PITCH,
        units::degree_t{filteredTarget.GetPitch()});

        if(DebugConstants::debugLimelight == true){
            frc::SmartDashboard::PutNumber("FilteredRange", filteredRange.value());
            frc::SmartDashboard::PutNumber("FilteredYaw", filteredTarget.GetYaw());
            frc::SmartDashboard::PutNumber("FilteredPitch", filteredTarget.GetPitch());
        }
      }
    }   
}

bool LimelightSubsystem::PhotonHasTarget(){
    return result.HasTargets();
}

double LimelightSubsystem::PhotonYaw(){
    return result.GetBestTarget().GetYaw();
}

double LimelightSubsystem::GetAprilTagtx()
{
    return AprilTagstx;
}


double LimelightSubsystem::GetAprilTagty()
{
    return AprilTagsty;
}

double LimelightSubsystem::GetNotetx()
{
    return Notetx;
}

double LimelightSubsystem::GetNotety()
{
    return Notety;
}

double LimelightSubsystem::FilteredPhotonYaw(){
    return filteredTarget.GetYaw();
}

