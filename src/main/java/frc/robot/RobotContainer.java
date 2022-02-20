// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.turret.AlignTurretWithOnboardPIDController;
import frc.robot.subsystems.turret.RunTurretWithJoystick;
import frc.robot.subsystems.turret.TurnToTargetClosedLoop;
import frc.robot.subsystems.turret.TurretSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final RunTurretWithJoystick runTurretWithJoystick = new RunTurretWithJoystick(turretSubsystem, joystick, 0.1);
  private final TurnToTargetClosedLoop turnToTargetClosedLoop = new TurnToTargetClosedLoop(turretSubsystem, limelightSubsystem);


  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {
    // new JoystickButton(joystick, XboxController.Button.kA.value).whenPressed(new AlignTurretWithOnboardPIDController(turretSubsystem, limelightSubsystem));
  }

  private void smartDashboardBindings() {
    SmartDashboard.putNumber("kP", 0.005);
    SmartDashboard.putNumber("kI", 0);
    SmartDashboard.putNumber("kD", 0);
  }

  public Command getAutonomousCommand() {
    return turnToTargetClosedLoop;
  }
  public void setDefaultCommands() {
    sparkMaxSubsystem.setDefaultCommand(joystickSparkMax);
  }

  
}
