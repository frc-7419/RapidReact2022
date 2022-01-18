/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  
  @Override
  public void robotPeriodic() {
    // RobotContainer.arcade.start();
    CommandScheduler.getInstance().run();
  }


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }
  
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running. 
    // Uncomment this once you have an auto command to run to make sure it doesnt keep running in teleop
    robotContainer.getAutonomousCommand().cancel();
    robotContainer.scheduleDefaultCommands();

  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
