package frc.robot.subsystems.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class FollowTarget extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private DriveBaseSubsystem driveBase;
  private LimelightSubsystem limelight;
  private PIDController pidController;
  
  private double kP;
  private double kI;
  private double kD;

  private double pidOutput;
  private double tx;
  private double ty;
  private double distanceToTarget;
  private double boost;

  private double velocityThreshold = 115;
  private boolean velocityBelow = false;

  public FollowTarget(DriveBaseSubsystem driveBase, LimelightSubsystem limelight) {
    this.driveBase = driveBase;
    this.limelight = limelight;
    addRequirements(driveBase, limelight);
  }

  @Override
  public void initialize() {

    kP = .016; // gets P coefficient from dashboard
    kI = 0;
    kD = 1; 
    pidController = new PIDController(kP, kI, kD);
    pidController.setSetpoint(0);
    pidController.setTolerance(1);
  }

  @Override
  public void execute() {
    SmartDashboard.putString("command status", "pid");

    tx = limelight.getTx();
    ty = limelight.getTy();

    pidOutput = pidController.calculate(tx);
    boost = Math.abs(pidOutput) / pidOutput * .05;
    pidOutput += boost;
    SmartDashboard.putNumber("pidoutput", pidOutput);
    driveBase.setLeftPower(-pidOutput);
    driveBase.setRightPower(pidOutput);

    distanceToTarget = (LimelightConstants.kTargetHeight - LimelightConstants.kCameraHeight) / Math.tan(Math.toRadians(ty));
    distanceToTarget = 1.426*distanceToTarget - 52.372; // based on linear regression, hopefully accurate
    SmartDashboard.putNumber("distance", distanceToTarget);

    if(Math.abs(driveBase.getLeftVelocity()) < velocityThreshold){
      if(Math.abs(driveBase.getRightVelocity()) < velocityThreshold){
        velocityBelow = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.setAll(0);
  }

  @Override
  public boolean isFinished() {
    return velocityBelow && pidController.atSetpoint();
  }
}
 