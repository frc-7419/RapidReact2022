package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class AlignTurret extends CommandBase {

  private TurretSubsystem turretSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private PIDController pidController;
  
  private double kP;
  private double kI;
  private double kD;

  private double pidOutput;
  private double tx;
  private double tv;

  private double initialAngle = Double.NaN;

  public AlignTurret(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(turretSubsystem, limelightSubsystem);
  }
  public AlignTurret(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, double initialAngle) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(turretSubsystem, limelightSubsystem);
    this.initialAngle = initialAngle;
  }

  @Override
  public void initialize() {
    if (initialAngle != Double.NaN) turretSubsystem.setAngle(this.initialAngle);
    SmartDashboard.putString("command status", "pid");

    kP = SmartDashboard.getNumber("kP", PIDConstants.TurretKp);
    kI = SmartDashboard.getNumber("kI", PIDConstants.TurretKi);
    kD = SmartDashboard.getNumber("kD", PIDConstants.TurretKd); 

    pidController = new PIDController(kP, kI, kD);
    pidController.setSetpoint(0);
    pidController.setTolerance(1);
  }

  @Override
  public void execute() {
    tx = limelightSubsystem.getTx();
    tv = limelightSubsystem.getTv();

    kP = SmartDashboard.getNumber("kP", PIDConstants.TurretKp);
    kI = SmartDashboard.getNumber("kI", PIDConstants.TurretKi);
    kD = SmartDashboard.getNumber("kD", PIDConstants.TurretKd); 

    if (tv == 1.0) {
      pidController = new PIDController(kP, kI, kD);
      pidOutput = pidController.calculate(tx);
      // SmartDashboard.putNumber("pid output", pidOutput);
      turretSubsystem.setPower(-pidOutput);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
 