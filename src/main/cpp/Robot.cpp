#include "Robot.h"

void Robot::RobotInit() {}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
    // ranAuto = true; 
  }
  // m_autoDriveStateMachine = m_container.GetAutoDriveStateMachine();
  // m_autoDriveStateMachine->Schedule();
  // m_autoAuxilaryStateMachine = m_container.GetAutoAuxilaryStateMachine();
  // m_autoAuxilaryStateMachine->Schedule();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.

  // m_container.SetRanAuto(ranAuto);
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand.reset();
  }
  // if (m_autoAuxilaryStateMachine) {
  //   m_autoAuxilaryStateMachine->Cancel();
  //   m_autoAuxilaryStateMachine.reset();
  // }  
  // if (m_autoDriveStateMachine) {
  //   m_autoDriveStateMachine->Cancel();
  //   m_autoDriveStateMachine.reset();
  // }
  m_stateMachine = m_container.GetAuxilaryStateMachine();
  m_stateMachine->Schedule();
  //m_driveStateMachine = m_container.GetDriveStateMachine();
  //m_driveStateMachine->Schedule();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
