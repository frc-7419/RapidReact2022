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
    
    new JoystickButton(joystick, XboxController.Button.kX.value)
      .and(new JoystickButton(joystick, XboxController.Button.kY.value))
      .toggleWhenActive(new GetToTargetVelocity(shooterSubsystem, 0, 0));

  }

  private void smartDashboardBindings() {
    SmartDashboard.putNumber("tTargetVelocity", 0);
    SmartDashboard.putNumber("bTargetVelocity", 0);

    SmartDashboard.putNumber("bKp", PIDConstants.BottomShooterkP);
    SmartDashboard.putNumber("bKi", PIDConstants.BottomShooterkI);

    SmartDashboard.putNumber("tKp", PIDConstants.TopShooterkP);
    SmartDashboard.putNumber("tKi", PIDConstants.TopShooterkI);
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

  public void setDefaultCommands(){
  }
}
