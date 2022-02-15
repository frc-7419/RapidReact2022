// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController joystick = new XboxController(0);
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();

  private final TurnWithGyroClosedLoop turnWithGyroClosedLoop = new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem);
  

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    smartDashboardBindings();
  }


  private void configureButtonBindings() {}

  private void smartDashboardBindings() {
    SmartDashboard.putNumber("kp", PIDConstants.GyrokP);
    SmartDashboard.putNumber("ki", PIDConstants.GyrokI);
    SmartDashboard.putNumber("kd", PIDConstants.GyrokD);
    SmartDashboard.putNumber("kMaxVelocity", 10);
    SmartDashboard.putNumber("kMaxAcc", 5);
    SmartDashboard.putNumber("target", 180);
    SmartDashboard.putBoolean("at setpoint", false);
  }

  // uncomment when u need to use this
  public Command getAutonomousCommand() {
    return turnWithGyroClosedLoop;
  }

  // set default commands here
  public void setDefaultCommands() {
    
  }
}
