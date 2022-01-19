// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.limelight.TurnOffLED;
import frc.robot.subsystems.limelight.TurnToTargetOpenLoop;
import frc.robot.subsystems.limelight.TurnToTargetWithGyro;
import frc.robot.subsystems.limelight.TurnToTargetClosedLoop;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.limelight.FollowTarget;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController joystick = new XboxController(0);

  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();

  // private final TurnOffLED turnOffLED = new TurnOffLED(limelightSubsystem);
  private final TurnToTargetClosedLoop turnToTargetClosedLoop = new TurnToTargetClosedLoop(driveBaseSubsystem, limelightSubsystem);
  private final FollowTarget followTarget = new FollowTarget(driveBaseSubsystem, limelightSubsystem);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick, driveBaseSubsystem, 0.5, 0.5, 0.5, 0.5);
  private final TurnToTargetWithGyro turnToTargetWithGyro = new TurnToTargetWithGyro(driveBaseSubsystem, limelightSubsystem, gyroSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // A button: limelight turn to target open loop test
    new JoystickButton(joystick, XboxController.Button.kA.value)
    .whenPressed(new TurnToTargetOpenLoop(driveBaseSubsystem, limelightSubsystem, 0.1));

    //X button: limelight turn to target closed loop test
    new JoystickButton(joystick, XboxController.Button.kX.value)
    .whenPressed(new TurnToTargetClosedLoop(driveBaseSubsystem, limelightSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */

  // uncomment when u need to use this
  public Command getAutonomousCommand() {
    return turnToTargetClosedLoop;
  }

  // schedule default commands here
  public void setDefaultCommands(){
    limelightSubsystem.setDefaultCommand(followTarget);
  }

  
}
