// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class AlignTurretWithPositionClosedLoop extends CommandBase {
  private TurretSubsystem turretSubsystem = new TurretSubsystem();
  private LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kF = 0;

  private double setpoint;
  private double tolerance = 0.2; // in inches (placeholder)
  private double turretPercentOutput;
  private double turretPosition;
  private double turretVelocity;

  public AlignTurretWithPositionClosedLoop(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // initializing turret talon
    turretSubsystem.getTurretMotor().restoreFactoryDefaults();

    // velocities in rpm
    turretSubsystem.getTurretPIDController().setSmartMotionMaxVelocity(15000, 0);
    // turretSubsystem.getTurretPIDController().setSmartMotionMinOutputVelocity(minVel, 0);
    turretSubsystem.getTurretPIDController().setSmartMotionMaxAccel(6000, 0);

    // set tolerance
    turretSubsystem.getTurretPIDController().setSmartMotionAllowedClosedLoopError(UnitConversions.inchesToTicks(tolerance, RobotConstants.turretRadius , 100/12, 4096), 0);

     // reset sensor position
     turretSubsystem.getTurretEncoder().setPosition(0);

    // setting PIDF constants
    turretSubsystem.setPIDFConstants(kP, kI, kD, kF);

    // initialize setpoint
    setpoint = UnitConversions.inchesToTicks(UnitConversions.thetaToInches(limelightSubsystem.getTx(), RobotConstants.turretRadius), RobotConstants.turretRadius, 100/12, 4096);
    turretSubsystem.getTurretPIDController().setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretPercentOutput = turretSubsystem.getTurretMotor().get();
    turretPosition = turretSubsystem.getTurretEncoder().getPosition();
    turretVelocity = turretSubsystem.getTurretEncoder().getVelocity();

    SmartDashboard.putNumber("turret position", turretPosition);
    SmartDashboard.putNumber("turret percent output", turretPercentOutput);
    SmartDashboard.putNumber("turret velocity (rpm)",  turretVelocity);
    SmartDashboard.putNumber("closed loop error", (setpoint - turretPosition));
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
    return Math.abs(turretPercentOutput) < 0.01;
  }
}
