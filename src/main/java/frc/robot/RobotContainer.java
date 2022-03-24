// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.team7419.joystick.DoubleButton;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.GetToTargetVelocity;

public class RobotContainer {
  private final XboxController joystick = new XboxController(0);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {
    // new JoystickButton(joystick, XboxController.Button.kY.value).toggleWhenPressed(new GetToTargetVelocity(shooterSubsystem, 1000, 1000));
    
    new DoubleButton(
      new JoystickButton(joystick, XboxController.Button.kX.value), 
      new JoystickButton(joystick, XboxController.Button.kY.value))
      .toggleWhenPressed(new GetToTargetVelocity(shooterSubsystem, 2000, 2000, 0.05, 0.05));

  }

  private void smartDashboardBindings() {
    SmartDashboard.putNumber("tTargetRV", 2000);
    SmartDashboard.putNumber("bTargetRV", 2000);
    SmartDashboard.putNumber("ShooterKp", PIDConstants.ShooterkP);
    SmartDashboard.putNumber("ShooterKi", PIDConstants.ShooterkI);
    SmartDashboard.putNumber("bKf", PIDConstants.ShooterkF);
    SmartDashboard.putNumber("tKf", PIDConstants.ShooterkF);
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

  public void setDefaultCommands(){
  }
}
