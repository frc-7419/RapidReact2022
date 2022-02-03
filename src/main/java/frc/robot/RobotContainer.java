// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.RunShooterWithJoystick;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.shooter.BasicShooterSubsystem;
import frc.robot.subsystems.shooter.GetToTargetVelocity;

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
  private final BasicShooterSubsystem shooterBasicSubsystem = new BasicShooterSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final RunShooterWithJoystick runShooterWithJoystick = new RunShooterWithJoystick(shooterBasicSubsystem, joystick);
  private final GetToTargetVelocity getToTargetVelocity = new GetToTargetVelocity(shooterSubsystem, limelightSubsystem);

  public RobotContainer() {
    configureButtonBindings();

    smartDashboardBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick, XboxController.Button.kY.value).whileHeld(new GetToTargetVelocity(shooterSubsystem, limelightSubsystem));
   
  }

  private void smartDashboardBindings() {
    SmartDashboard.putNumber("targetRPM", 1000);
    SmartDashboard.putNumber("shooterKp", PIDConstants.ShooterkP);
    SmartDashboard.putNumber("shooterKi", PIDConstants.ShooterkI);
  }

  public Command getAutonomousCommand() {
    return runShooterWithJoystick;
  }

  public void setDefaultCommands(){
    shooterBasicSubsystem.setDefaultCommand(runShooterWithJoystick);
  }
}
