// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.ArrowRunShooterWithJoystick;
import frc.robot.subsystems.shooter.BasicShooterSubsystem;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.GetToTargetVelocity;

public class RobotContainer {
  private final XboxController joystick = new XboxController(0);
  private final BasicShooterSubsystem basicShooterSubsystem = new BasicShooterSubsystem();
  
  private final ArrowRunShooterWithJoystick arrowRunShooterWithJoystick = new ArrowRunShooterWithJoystick(basicShooterSubsystem, joystick);

  public RobotContainer() {
    configureButtonBindings();

    smartDashboardBindings();
  }

  private void configureButtonBindings() {
    // new JoystickButton(joystick, XboxController.Button.kY.value).toggleWhenPressed(new GetToTargetVelocity(shooterSubsystem, 1000));
   
  }

  private void smartDashboardBindings() {
    // SmartDashboard.putNumber("targetRPM", 1000);
    // SmartDashboard.putNumber("shooterKp", PIDConstants.ShooterkP);
    // SmartDashboard.putNumber("shooterKi", PIDConstants.ShooterkI);
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

  public void setDefaultCommands(){
    // shooterBasicSubsystem.setDefaultCommand(runShooterWithJoystick);
    basicShooterSubsystem.setDefaultCommand(arrowRunShooterWithJoystick);
  }
}
