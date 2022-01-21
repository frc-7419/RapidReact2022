package frc.robot.subsystems.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;

public class TurnToTargetWithGyro extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private DriveBaseSubsystem driveBaseSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private PIDController pidController;
  private GyroSubsystem gyroSubsystem;
  
  private double kP;
  private double kI;
  private double kD;

  private double pidOutput;
  private double initAngle;
  private double gyroAngle;

  private double velocityThreshold = 115;
  private boolean velocityBelow = false;

  public TurnToTargetWithGyro(DriveBaseSubsystem driveBaseSubsystem, LimelightSubsystem limelightSubsystem, GyroSubsystem gyroSubsystem) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    addRequirements(driveBaseSubsystem, limelightSubsystem, gyroSubsystem);
  }

  @Override
  public void initialize() {

    kP = .016; // gets P coefficient from dashboard ah yes dashboard
    kI = 0;
    kD = 1;
    pidController = new PIDController(kP, kI, kD);

    initAngle = limelightSubsystem.getTx();
    gyroAngle = gyroSubsystem.getGyroAngle();

    pidController.setSetpoint(initAngle + gyroAngle);
    pidController.setTolerance(1);

  }

  @Override
  public void execute() {
    SmartDashboard.putString("command status", "pid");

    gyroAngle = gyroSubsystem.getGyroAngle();

    pidOutput = pidController.calculate(gyroAngle);
    
    SmartDashboard.putNumber("pidoutput", pidOutput);

    driveBaseSubsystem.setLeftPower(-pidOutput);
    driveBaseSubsystem.setRightPower(pidOutput);

    if(Math.abs(driveBaseSubsystem.getLeftVelocity()) < velocityThreshold){
      if(Math.abs(driveBaseSubsystem.getRightVelocity()) < velocityThreshold){
        velocityBelow = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setAll(0);
  }

  @Override
  public boolean isFinished() {
    return velocityBelow && pidController.atSetpoint();
  }
}
 