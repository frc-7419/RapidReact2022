// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.turret.AlignTurretWithOnboardPIDController;
import frc.robot.subsystems.turret.TurretSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.encoders.JoystickSparkMax;
import frc.robot.subsystems.encoders.SparkMaxSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.limelight.TurnToTargetClosedLoop;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);
  private final SparkMaxSubsystem sparkSubsystem = new SparkMaxSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final JoystickSparkMax joystickSparkMax = new JoystickSparkMax(sparkSubsystem, joystick);
  private final TurnToTargetClosedLoop turnToTargetClosedLoop = new TurnToTargetClosedLoop(sparkSubsystem,limelightSubsystem);

  private final TurretSubsystem turretSubsystem = new TurretSubsystem();

  private final AlignTurretWithOnboardPIDController alignTurretWithOnboardPIDController = new AlignTurretWithOnboardPIDController(turretSubsystem, limelightSubsystem);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick, XboxController.Button.kA.value).whenPressed(new AlignTurretWithOnboardPIDController(turretSubsystem, limelightSubsystem));
  }

  public Command getAutonomousCommand() {
    return turnToTargetClosedLoop;
  }
  public void setDefaultCommands() {
    // turretSubsystem.setDefaultCommand(runTurret);
  }

  
}
