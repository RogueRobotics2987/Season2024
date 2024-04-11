#include "subsystems/LightSubsystem.h"

//Yellow - when intake is on
//Green - loaded
//purple - locked on and ready to shoot.

LightSubsystem::LightSubsystem()
{

}


void LightSubsystem::Periodic()
{

}


void LightSubsystem::SetLightsGreen()
{
    pin2.Set(0);
    pin3.Set(1);
    pin4.Set(0);
}

void LightSubsystem::SetLightsYellow()
{
    pin2.Set(0);
    pin3.Set(0);
    pin4.Set(1);
}

void LightSubsystem::SetLightsPurple()
{
    pin2.Set(1);
    pin3.Set(1);
    pin4.Set(0);
}

void LightSubsystem::SetNoColor()
{
    pin2.Set(1);
    pin3.Set(0);
    pin4.Set(1);
}

void LightSubsystem::SetColorChase()
{
    pin2.Set(1);
    pin3.Set(1);
    pin4.Set(1);
}

void LightSubsystem::SetFlashPurple()
{
    pin2.Set(0);
    pin3.Set(1);
    pin4.Set(1);
}

void LightSubsystem::SetLightsInitial()
{
    pin2.Set(1);
    pin3.Set(0);
    pin4.Set(0);
}