#include "subsystems/ShooterSubsystem.h"


ShooterSubsystem::ShooterSubsystem(){
ShooterSolenoid1 = new frc::DoubleSolenoid(frc::PneumaticsModuleType::REVPH, 0, 1); 
ShooterSolenoid2 = new frc::DoubleSolenoid(frc::PneumaticsModuleType::REVPH, 2, 3); 
ShooterSolenoid3 = new frc::DoubleSolenoid(frc::PneumaticsModuleType::REVPH, 4, 5); 
ShooterSolenoid4 = new frc::DoubleSolenoid(frc::PneumaticsModuleType::REVPH, 6, 7); 
ShooterSolenoid5 = new frc::DoubleSolenoid(frc::PneumaticsModuleType::REVPH, 8, 9); 
}

void ShooterSubsystem::Periodic() {}
void ShooterSubsystem::Close(int SolenoidNum){
    

    if(SolenoidNum==1){
        ShooterSolenoid1->Set(frc::DoubleSolenoid::kForward);
    } else if(SolenoidNum==2){
        ShooterSolenoid2->Set(frc::DoubleSolenoid::kForward);
    } else if(SolenoidNum==3){
        ShooterSolenoid3->Set(frc::DoubleSolenoid::kForward);
    } else if(SolenoidNum==4){
        ShooterSolenoid4->Set(frc::DoubleSolenoid::kForward);
    } else if(SolenoidNum==5){
        ShooterSolenoid5->Set(frc::DoubleSolenoid::kForward);
    }
}

void ShooterSubsystem::Open(int SolenoidNum){ 

    if(SolenoidNum==1){
        ShooterSolenoid1->Set(frc::DoubleSolenoid::kReverse);
    } else if(SolenoidNum==2){
        ShooterSolenoid2->Set(frc::DoubleSolenoid::kReverse);
    } else if(SolenoidNum==3){
        ShooterSolenoid3->Set(frc::DoubleSolenoid::kReverse);
    } else if(SolenoidNum==4){
        ShooterSolenoid4->Set(frc::DoubleSolenoid::kReverse);
    } else if(SolenoidNum==5){
        ShooterSolenoid5->Set(frc::DoubleSolenoid::kReverse);
    }
} 

