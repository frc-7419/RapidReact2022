/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;

public class Robot extends TimedRobot {
  private RobotContainer robotContainer;

  private DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  
  @Override
  public void robotPeriodic() {
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
    robotContainer.getAutonomousCommand().cancel();
    robotContainer.setDefaultCommands();

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
