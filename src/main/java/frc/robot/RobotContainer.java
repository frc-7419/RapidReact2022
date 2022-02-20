// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeDefault;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.intake.DeployIntakeWithPneumatics;
import frc.robot.subsystems.pneumatics.CompressorSubsystem;
import frc.robot.subsystems.pneumatics.RunSolenoid;
import frc.robot.subsystems.pneumatics.SolenoidForwardAndReverse;
import frc.robot.subsystems.pneumatics.SolenoidSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);

  private final SolenoidSubsystem solenoid = new SolenoidSubsystem();
  private final SolenoidForwardAndReverse solenoidForwardAndReverse = new SolenoidForwardAndReverse(solenoid);
  private final CompressorSubsystem compressorSubsystem = new CompressorSubsystem();
  private final RunSolenoid runSolenoid = new RunSolenoid(solenoid, joystick);
  private final DeployIntakeWithPneumatics deployIntakeWithPneumatics = new DeployIntakeWithPneumatics(solenoid, joystick);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final IntakeDefault intakeDefault = new IntakeDefault(intakeSubsystem, joystick);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // new JoystickButton(joystick, XboxController.Button.kX.value).whenPressed(new RunPneumaticsSystem(solenoid, false));
    // new JoystickButton(joystick, XboxController.Button.kY.value).whenPressed(new RunPneumaticsSystem(solenoid, true));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }
  public void setDefaultCommands(){
    intakeSubsystem.setDefaultCommand(intakeDefault);
    this.solenoid.setDefaultCommand(deployIntakeWithPneumatics);
  }

  
}
