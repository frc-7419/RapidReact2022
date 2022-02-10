// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.autos.Period2Auton;
import frc.robot.subsystems.autos.TemplateSequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
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

  /* suggestions from Karan:
  instantiate both driveBaseSubsystem and gyroSubsystem
  */
  public final DriveBaseSubsystem driveBaseSubsystem;
  public final GyroSubsystem gyroSubsystem;
  private final Period2Auton period2Auton ;

  // instantiate your auto commands here
  

  public RobotContainer() {
    // Configure the button bindings
    this.driveBaseSubsystem = new DriveBaseSubsystem();
    this.gyroSubsystem = new GyroSubsystem();
    this.period2Auton = new Period2Auton(driveBaseSubsystem, gyroSubsystem);

    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {}

  private void smartDashboardBindings() {}

  // uncomment when u need to use this
  public Command getAutonomousCommand() {
    return period2Auton;
  }

  // set default commands here
  public void setDefaultCommands() {
    
  }
}
