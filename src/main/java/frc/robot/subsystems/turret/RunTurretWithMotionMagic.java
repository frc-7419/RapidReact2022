// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class RunTurretWithMotionMagic extends CommandBase {
  private TurretSubsystem turretSubsystem = new TurretSubsystem();
  private LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kF = 0;

  private double setpoint;
  private double tolerance = 10; // in ticks (placeholder)
  private double turretPercentOutput;

  public RunTurretWithMotionMagic(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // initializing turret talon
    turretSubsystem.getTurretTalon().configFactoryDefault();

    turretSubsystem.getTurretTalon().configMotionCruiseVelocity(15000, 0);
    turretSubsystem.getTurretTalon().configMotionAcceleration(6000, 0); 
    turretSubsystem.getTurretTalon().configAllowableClosedloopError(0, tolerance);

     // reset sensor position
    turretSubsystem.getTurretTalon().setSelectedSensorPosition(0);

    // setting PIDF constants
    turretSubsystem.setPIDFConstants(kP, kI, kD, kF);

    // initialize setpoint
    setpoint = UnitConversions.thetaToTicks(limelightSubsystem.getTx(), RobotConstants.turretRadius);
    turretSubsystem.getTurretTalon().set(ControlMode.MotionMagic, setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("turret position", turretSubsystem.getTurretTalon().getSelectedSensorPosition(0));

    turretPercentOutput = turretSubsystem.getTurretTalon().getMotorOutputPercent();

    SmartDashboard.putNumber("turret percent output", turretPercentOutput);
    SmartDashboard.putNumber("closed loop error", turretSubsystem.getTurretTalon().getClosedLoopError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // threshold: motor output < 0.01
    return (Math.abs(turretPercentOutput) < 0.01);
  }
}
