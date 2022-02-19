package frc.robot.subsystems.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.spark.SparkMaxSubsystem;

public class TurnToTargetClosedLoop extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private SparkMaxSubsystem sparkMaxSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private PIDController pidController;
  
  private double kP;
  private double kI;
  private double kD;

  private double pidOutput;
  private double tx;
  private double tv;

  public TurnToTargetClosedLoop(SparkMaxSubsystem sparkMaxSubsystem, LimelightSubsystem limelightSubsystem) {
    this.sparkMaxSubsystem = sparkMaxSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(sparkMaxSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("command status", "pid");

    kP = SmartDashboard.getNumber("kP", 0.08);
    kI = SmartDashboard.getNumber("kI", 0);
    kD = SmartDashboard.getNumber("kD", 0); 

    pidController = new PIDController(kP, kI, kD);
    pidController.setSetpoint(0);
    pidController.setTolerance(1);
  }

  @Override
  public void execute() {
    tx = limelightSubsystem.getTx();
    tv = limelightSubsystem.getTv();

    kP = SmartDashboard.getNumber("kP", 0.08);
    kI = SmartDashboard.getNumber("kI", 0);
    kD = SmartDashboard.getNumber("kD", 0); 

    if (tv == 1.0) {
      pidController = new PIDController(kP, kI, kD);
      pidOutput = pidController.calculate(tx);
      SmartDashboard.putNumber("pid output", pidOutput);
      sparkMaxSubsystem.setPower(-pidOutput);
    }
  }

  @Override
  public void end(boolean interrupted) {
    sparkMaxSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    // return pidController.atSetpoint();
    return false;
  }
}
 