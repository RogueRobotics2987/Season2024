// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

// ClimberSubsystem::ClimberSubsystem(
//   int m_MotorController,
//   rev::SparkRelativeEncoder::Type m_EncoderType,
//   int m_counts_per_rev
// )

// {

// }

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() 
{

}


void ClimberSubsystem::startClimber() 
{
    m_climberMoter.Set(0.5);  
}

void ClimberSubsystem::stopClimber() 
{
    m_climberMoter.Set(0.0);
}




/*
    "What the Flibity Bajingaas?" said the flunky binky older gentleman when he couldn't find his reading glasses.
    "Hey pops, what's all this commotion about?" said the his son, Bob. 
    The older gentleman responded with, "I'm sorry to worry you son, but I can't seem to find my poopity scoop glasses. It's really pushing my buttons, I'm sorry." 
    Bob: "Don't worry old sport, I'll help you find them!"
    5 seconds later...
    Bob: "Bruh, how are you this skibbidy pop?"
    Gramps: "Oh, I'm sorry son. I know I'm a bit sus, but I really can't find them."
    Bob: "BRUH, you goblitering boob! They're one you stanky head!"
    Gramps: "NOOOOOOOO! I can't believe it! Please don't vote me out I have a wife and son!"
    Bob: "I know, I am your son after all. However I can't overlook this fatal mistake, red. I'm not skipping on this one. GET THE HECK OUT OF HERE!"
    Gramps: "NOOOOO! Fiddle sticks! Rabloop! Shucks! AHHHHasdasjfk."

    Gramps was the imposter.

    The Ebster - FHS Senior
    February 10, 2024.
*/