// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.servo.ServoSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretWithPositionClosedLoop;
import frc.robot.subsystems.turret.TurretSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.colorSensor.ColorSensorSubsystem;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.limitswitch.LimitswitchSubsystem;
import frc.robot.subsystems.potentiometer.PotentiometerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);

  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem();
  private final LimitswitchSubsystem limitSwitchSubsystem = new LimitswitchSubsystem();

  private final TurretSubsystem turretSubsystem = new TurretSubsystem();

  private final AlignTurretWithPositionClosedLoop runTurret = new AlignTurretWithPositionClosedLoop(turretSubsystem, limelightSubsystem);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
   
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }
  public void setDefaultCommands(){
    turretSubsystem.setDefaultCommand(runTurret);
  }

  
}
