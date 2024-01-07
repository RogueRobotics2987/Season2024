#include "RobotContainer.h"


using namespace DriveConstants;


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
      std::cout << "cout in robot container" << std::endl;


  // Configure the button bindings
  ConfigureButtonBindings();
  m_drive.ZeroHeading(); //resets the heading on the gyro


m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
      bool noJoystickInput = false; //checks if there is any joystick input (if true the wheels will go to the the 45 degree (X) position)
      double safeX = Deadzone(m_driverController.GetLeftX());
      double safeY =  Deadzone(m_driverController.GetLeftY());
      double safeRot = Deadzone(m_driverController.GetRightX());


      bool fieldOrientated;
      if (m_driverController.GetRawAxis(3)> 0.15){ //if the right trigger is pulled
        fieldOrientated = false; //robot orientated driving
      }
      if (m_driverController.GetRawAxis(3)< 0.15){ //if the right trigger is not pulled
        fieldOrientated = true; //field orientated driving
      }


      if ((safeX == 0) && (safeY == 0) && (safeRot == 0)) {
        noJoystickInput = true; //the wheels will move to the 45 degree (X) position
      }


      m_drive.Drive(units::meters_per_second_t(
                    -safeY * AutoConstants::kMaxSpeed),
                    units::meters_per_second_t(
                    -safeX * AutoConstants::kMaxSpeed),
                    units::radians_per_second_t(
                    -safeRot * std::numbers::pi * 1.5),
                    fieldOrientated,
                    noJoystickInput);
      },{&m_drive}));

m_limePose.SetDefaultCommand(LimeLightCmd(m_limePose));

}



void RobotContainer::ConfigureButtonBindings() {
  //Resets the heading of the gyro. In other words, it resets which way the robot thinks is the front
  frc2::JoystickButton(&m_driverController, 5).OnTrue(m_drive.ZeroHeading());


  //Robot slides right (when front is away from the drivers)
  frc2::JoystickButton(&m_driverController, 1).WhileTrue(m_drive.Twitch(true));


  //Robot slides left (when front is away from the drivers)
  frc2::JoystickButton(&m_driverController, 2).WhileTrue(m_drive.Twitch(false));
}


float RobotContainer::Deadzone(float x){
  if ((x < 0.1) &&  (x > -0.1)){
    x=0;
  } else if (x >= 0.1){
    x = x - 0.1;
  } else if (x <= -0.1){
    x = x + 0.1;
  }
  return(x);
}


frc2::Command* RobotContainer::GetAutonomousCommand() {
  return NULL;
}
