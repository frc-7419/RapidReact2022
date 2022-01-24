// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.colorSensor.ColorSensorSubsystem;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeDefault;
import frc.robot.subsystems.intake.IntakeSubsystem;
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
  private final PotentiometerSubsystem potentiometerSubsystem = new PotentiometerSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final IntakeDefault intakeDefault = new IntakeDefault(intakeSubsystem, joystick);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
   
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }
  public void setDefaultCommands(){
    intakeSubsystem.setDefaultCommand(intakeDefault);
  }

  
}
