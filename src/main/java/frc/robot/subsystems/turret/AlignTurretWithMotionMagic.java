// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.math.UnitConversions;
import com.team7419.math.UnitConversions.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class AlignTurretWithMotionMagic extends CommandBase {
  private TurretSubsystem turretSubsystem = new TurretSubsystem();
  private LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kF = 0;

  private double setpoint;
  private double tolerance = 10; // in ticks (placeholder)
  private double turretPercentOutput;

  public AlignTurretWithMotionMagic(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // initializing turret talon
    turretSubsystem.getTurretMotor().restoreFactoryDefaults();

    turretSubsystem.getTurretMotor().configMotionCruiseVelocity(15000, 0);
    turretSubsystem.getTurretMotor().configMotionAcceleration(6000, 0); 

    turretSubsystem.getTurretMotor().configAllowableClosedloopError(0, tolerance);

     // reset sensor position
    turretSubsystem.getTurretMotor().setSelectedSensorPosition(0);

    // setting PIDF constants
    turretSubsystem.setPIDFConstants(kP, kI, kD, kF);

    // initialize setpoint
    setpoint = UnitConversions.inchesToTicks(MotorType.SparkMAX, UnitConversions.thetaToInches(limelightSubsystem.getTx(), RobotConstants.turretRadius), RobotConstants.turretRadius);
    turretSubsystem.getTurretMotor().set(ControlMode.MotionMagic, setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("turret position", turretSubsystem.getTurretMotor().getSelectedSensorPosition(0));

    turretPercentOutput = turretSubsystem.getTurretMotor().getMotorOutputPercent();

    SmartDashboard.putNumber("turret percent output", turretPercentOutput);
    SmartDashboard.putNumber("closed loop error", turretSubsystem.getTurretMotor().getClosedLoopError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // threshold: motor output < 0.01
    return (Math.abs(turretPercentOutput) < 0.01);
  }
}
