// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.team7419.math.UnitConversions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class AlignTurretWithPositionPIDController extends CommandBase {
  private TurretSubsystem turretSubsystem = new TurretSubsystem();
  private LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  private PIDController pidController;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private double setpoint;
  private double tolerance = 0.5; // native units

  private double pidOutput;
  
  public AlignTurretWithPositionPIDController(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSubsystem.coast();
    
    // instantiate PIDController class
    pidController = new PIDController(kP, kI, kD);

    // zero the encoder position
    turretSubsystem.getTurretEncoder().setPosition(0);

     // initialize setpoint
    setpoint = UnitConversions.inchesToTicks(UnitConversions.thetaToInches(limelightSubsystem.getTx(), RobotConstants.turretRadius), RobotConstants.turretRadius, RobotConstants.turretGearRatio, 4096);
    pidController.setSetpoint(setpoint);
    pidController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidOutput = pidController.calculate(turretSubsystem.getTurretEncoder().getPosition());
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
