// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimelightSubsystem.h"

LimelightSubsystem::LimelightSubsystem() = default;

// This method will be called once per scheduler run
void LimelightSubsystem::Periodic()
{
    //works to read values from the network tables for limelight and pushes them back out to confirm we are reading values
    // botPose = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose", std::span<const double>({1 , 0, 0, 0, 0, 0}));

    //TODO Coment this back in eventually once limelights are put onto the robot

    //AprilTagstx = ;//nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("tx",0.0);
    //AprilTagsty = ;//nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumber("ty",0.0);
    Notetx = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("tx",0.0);
    Notety = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("ty",0.0);
    auto threeDPose = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose",std::vector<double>(6));
    auto threeDPose2 = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")->GetNumberArray("botpose blue",std::vector<double>(6));


    double distanceToTagInMeters = threeDPose[0];
    frc::SmartDashboard::PutNumber("Distance", distanceToTagInMeters);
    frc::SmartDashboard::PutNumber("Distance2", threeDPose2[0]);
    

    result = camera.GetLatestResult();
    hasTarget = result.HasTargets();

    if(hasTarget == true)
    {
        tempTargets = result.GetTargets();
        myTargets.assign(tempTargets.begin(), tempTargets.end());

        for(unsigned int i = 0; i < myTargets.size(); i++)
        {
            if(myTargets.at(i).GetFiducialId() == 4 || myTargets.at(i).GetFiducialId() == 7)
            {
                filteredTarget = myTargets.at(i);
                filteredTargetID = filteredTarget.GetFiducialId();
            }
        }
    }
}


bool LimelightSubsystem::PhotonHasTarget(){
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

double LimelightSubsystem::PhotonYaw(){
    double yaw = filteredTarget.GetYaw();
    return yaw;
}

double LimelightSubsystem::GetNotetx()
{
    return Notetx;
}

double LimelightSubsystem::GetNotety()
{
    return Notety;
}


double LimelightSubsystem::GetAprilTagtx()
{
    return AprilTagstx;
}

double LimelightSubsystem::GetAprilTagty()
{
    return AprilTagsty;
}