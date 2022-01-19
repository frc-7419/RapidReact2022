package frc.robot.subsystems.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class TurnToTargetOpenLoop extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private DriveBaseSubsystem driveBaseSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private PIDController pidController;
  
  private double tx;
  private double ty;
  private double power;
  private double distanceToTarget;

  public TurnToTargetOpenLoop(DriveBaseSubsystem driveBaseSubsystem, LimelightSubsystem limelightSubsystem, double power) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.power = power;
    addRequirements(driveBaseSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    int directionalCoefficient = 1;

    tx = limelightSubsystem.getTx();
    ty = limelightSubsystem.getTy();

    // calculate negative coefficient based on position of target
    if (tx > 0) {directionalCoefficient = -1;}

    // set power to motors
    driveBaseSubsystem.setRightPower(directionalCoefficient * power);
    driveBaseSubsystem.setLeftPower(-directionalCoefficient * power);

    // distanceToTarget = (Constants.kTargetHeight - RobotConstants.kCameraHeight) / Math.tan(Math.toRadians(ty));
    // distanceToTarget = 1.426*distanceToTarget - 52.372; // based on linear regression, hopefully accurate
    // SmartDashboard.putNumber("distance", distanceToTarget);
  }

  @Override
  public void end(boolean interrupted) {
    // set motors to zero when it ends
    driveBaseSubsystem.setAll(0);
  }

  @Override
  public boolean isFinished() {
    // end condition: when tx gets within .05
    return Math.abs(limelightSubsystem.getTx()) < 0.05; //threshold
  }
}
 