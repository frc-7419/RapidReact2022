package frc.robot.subsystems.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.encoders.SparkMaxSubsystem;

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
//   private double ty;
//   private double distanceToTarget;
//   private double boost;

//   private double velocityThreshold = 115;
//   private boolean velocityBelow = false;

  public TurnToTargetClosedLoop(SparkMaxSubsystem sparkMaxSubsystem, LimelightSubsystem limelightSubsystem) {
    this.sparkMaxSubsystem = sparkMaxSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(sparkMaxSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("command status", "pid");
    kP = .008; // gets P coefficient from dashboard
    kI = 0;
    kD = 0; 
    pidController = new PIDController(kP, kI, kD);
    pidController.setSetpoint(0);
    pidController.setTolerance(1);
  }

  @Override
  public void execute() {
    tx = limelightSubsystem.getTx();
    tv = limelightSubsystem.getTv();
    // if (tv==1){
    SmartDashboard.putString("command status", "pid");

    
    // ty = limelightSubsystem.getTy();

    pidOutput = pidController.calculate(tx);
    // boost = Math.abs(pidOutput) / pidOutput * .05;
    // pidOutput += boost;
    SmartDashboard.putNumber("pidoutput", pidOutput);
    sparkMaxSubsystem.setPower(-pidOutput);

    // distanceToTarget = (LimelightConstants.kTargetHeight - LimelightConstants.kCameraHeight) / Math.tan(Math.toRadians(ty));
    // distanceToTarget = 1.426*distanceToTarget - 52.372; // based on linear regression, hopefully accurate
    // SmartDashboard.putNumber("distance", distanceToTarget);
// }
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
 