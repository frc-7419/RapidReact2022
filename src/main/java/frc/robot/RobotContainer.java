// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.turret.AlignTurretWithOnboardPIDController;
import frc.robot.subsystems.turret.RunTurretWithREVMagneticSwitch;
import frc.robot.subsystems.turret.AlignTurret;
import frc.robot.subsystems.turret.TurretSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.revMagneticLimitSwitch.RevMagneticLimitSwitchSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final RevMagneticLimitSwitchSubsystem leftLimitSwitch = new RevMagneticLimitSwitchSubsystem();
  private final RevMagneticLimitSwitchSubsystem rightLimitSwitch = new RevMagneticLimitSwitchSubsystem();
  private final RunTurretWithREVMagneticSwitch runTurretWithREVMagneticSwitch = new RunTurretWithREVMagneticSwitch(turretSubsystem, joystick, leftLimitSwitch, rightLimitSwitch, 0.3);
  private final AlignTurret turnToTargetClosedLoop = new AlignTurret(turretSubsystem, limelightSubsystem);


  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {}

  private void smartDashboardBindings() {
    SmartDashboard.putNumber("kP", PIDConstants.TurretKp);
    SmartDashboard.putNumber("kI", PIDConstants.TurretKi);
    SmartDashboard.putNumber("kD", PIDConstants.TurretKd);
  }

  public Command getAutonomousCommand() {
    return turnToTargetClosedLoop;
  }
  public void setDefaultCommands() {
    turretSubsystem.setDefaultCommand(runTurretWithREVMagneticSwitch);
  }

  
}
