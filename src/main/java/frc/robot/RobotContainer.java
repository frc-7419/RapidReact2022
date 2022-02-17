// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.autos.Period7Auton;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.UnBrake;
import frc.robot.subsystems.gyro.GyroSubsystem;
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
  private final Period7Auton period7Auton = new Period7Auton(driveBaseSubsystem, gyroSubsystem);

  private UnBrake unBrake = new UnBrake(driveBaseSubsystem);


  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick, driveBaseSubsystem, 0.6, 0.6, 0.6, 0.6);

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {
    // new JoystickButton(joystick, XboxController.Button.kX.value).whenPressed(unBrake);
  }

  private void smartDashboardBindings() {}

  // uncomment when u need to use this
  public Command getAutonomousCommand() {
    return period7Auton;

  }
  public Command getDefaultCommand(){
    return arcadeDrive;
  }
  // set default commands here
  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
  }
}
