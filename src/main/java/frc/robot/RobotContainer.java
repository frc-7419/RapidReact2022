// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.servo.ServoSubsystem;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.limelight.TurnLimelightWithServoToTyClosedLoop;
import frc.robot.subsystems.limelight.TurnLimelightWithServoToTyOpenLoop;
import frc.robot.subsystems.limelight.TurnLimelightWithServoToTyOpenLoop;
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
  private final ServoSubsystem servoSubsystem = new ServoSubsystem();

  private final TurnLimelightWithServoToTyOpenLoop turnLimelightWithServoToTyOpenLoop = new TurnLimelightWithServoToTyOpenLoop(servoSubsystem, limelightSubsystem, 70);
  private final TurnLimelightWithServoToTyClosedLoop turnLimelightWithServoToTyClosedLoop = new TurnLimelightWithServoToTyClosedLoop(servoSubsystem, limelightSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }

  // schedule default commands here
  public void setDefaultCommands() {
    servoSubsystem.setDefaultCommand(turnLimelightWithServoToTyOpenLoop);
  }


  // uncomment when u need to use this
  public Command getAutonomousCommand() {
    return turnLimelightWithServoToTyOpenLoop;
  }
}
