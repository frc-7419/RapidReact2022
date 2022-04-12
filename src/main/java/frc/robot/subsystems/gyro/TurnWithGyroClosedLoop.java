package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class TurnWithGyroClosedLoop extends CommandBase {
  
  private DriveBaseSubsystem driveBaseSubsystem;
  private GyroSubsystem gyroSubsystem;
  private double target;
  private double kP;
  private double kI;
  private double kD;
  private PIDController pidController;
  private double pidOutput;
  private double initAngle;
  private double tolerance;

  /**
   * LEFT IS POSITIVE
   * @param driveBase
   * @param gyro
   * @param angle
   */
  public TurnWithGyroClosedLoop(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, double tolerance, double target) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    this.target = target;
    this.tolerance = tolerance;
  }

  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
    initAngle = gyroSubsystem.getGyroAngle();
    kP = SmartDashboard.getNumber("kP", PIDConstants.GyrokP);
    kI = SmartDashboard.getNumber("kI", PIDConstants.GyrokI);
    kD = SmartDashboard.getNumber("kD", PIDConstants.GyrokD);
    pidController = new PIDController(kP, kI, kD);
    tolerance = SmartDashboard.getNumber("tolerance", tolerance);
    target = SmartDashboard.getNumber("target", target);
    pidController.setSetpoint(initAngle + target);
    pidController.setTolerance(tolerance); 
  } 

  @Override
  public void execute() {
    SmartDashboard.putNumber("gyro turn error", pidController.getPositionError());
    SmartDashboard.putBoolean("at setpoint", pidController.atSetpoint());
    pidOutput = pidController.calculate(gyroSubsystem.getGyroAngle());
    driveBaseSubsystem.setLeftPower(-pidOutput);
    driveBaseSubsystem.setRightPower(pidOutput);
    SmartDashboard.putNumber("robot turned", gyroSubsystem.getGyroAngle() - initAngle);
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.stop();
    driveBaseSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
