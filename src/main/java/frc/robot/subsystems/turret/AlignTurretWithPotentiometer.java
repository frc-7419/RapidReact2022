// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class AlignTurretWithPotentiometer extends CommandBase {
  private TurretSubsystem turretSubsystem = new TurretSubsystem();
  private LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private AnalogPotentiometer turretPotentiometer;

  private PIDController pidController;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private double setpoint;
  private double tolerance = 0.5;

  private double pidOutput;
  
  public AlignTurretWithPotentiometer(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    turretPotentiometer = new AnalogPotentiometer(1, 180, 0);
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSubsystem.coast();

    // instantiate PIDController class
    pidController = new PIDController(kP, kI, kD);
    setpoint = limelightSubsystem.getTx();
    pidController.setSetpoint(turretPotentiometer.get() + setpoint);
    pidController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidOutput = pidController.calculate(turretPotentiometer.get());
    turretSubsystem.setPower(pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setPower(0);
    turretSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
