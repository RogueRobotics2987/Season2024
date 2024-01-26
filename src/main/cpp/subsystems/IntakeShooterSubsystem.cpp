// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeShooterSubsystem.h"

IntakeSubsystem::IntakeSubsystem() = default;

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
    switch (state) {
    case EMPTY:     // turn everything off
        m_intakeMotor1->Set(0.0); 
        m_intakeMotor2->Set(0.0);
        m_intakeMotor3->Set(0.0);

        m_magMotor1->Set(0.0);
        m_magMotor2->Set(0.0);

        m_shooterMotor1->Set(0.0);
        m_shooterMotor2->Set(0.0);   
        

        if(orangeCheerio == true){
            state = PICKUP;   
        } 

        break;
    
    case PICKUP:    // start intake and magazine
        m_intakeMotor1->Set(0.5);
        m_intakeMotor2->Set(0.5);
        m_intakeMotor3->Set(0.5);   // TODO: switch direction based on robot direction

        if(detectiveOrange1 == true || detectiveOrange2 == true){
            m_magMotor1->Set(0.5);
            m_magMotor2->Set(0.5);

        }

        if(orangeCheerio == false){
            state = EMPTY;

        } else if(eatenCheerio == true){
            state = LOADED;
        }

        break;

    /*case MAGAZINE:

        break;
    */
    case LOADED:    // self explanitory
        m_intakeMotor1->Set(0.0);
        m_intakeMotor2->Set(0.0);
        m_intakeMotor3->Set(0.0);

        m_magMotor1->Set(0.0);
        m_magMotor2->Set(0.0);

        orangeCheerio = false;


        if(warmMilk == true){
            state = SHOOTER_WARMUP;

        } else if(spillMilk == true){
            state = DROP_WARMUP;
            
        }

        break;

    case SHOOTER_WARMUP:

        break;

    case SHOOT:

        break;
    
    case DROP_WARMUP:

        break;
    
    case DROP:

        break;
    
    default:
        state = EMPTY;
        break;
    }

}
