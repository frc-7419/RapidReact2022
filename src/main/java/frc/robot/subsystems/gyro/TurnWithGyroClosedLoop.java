package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class TurnWithGyroClosedLoop extends CommandBase {
  
  private DriveBaseSubsystem driveBase;
  private GyroSubsystem gyroSubsystem;
  private double target;
  private double kP;
  private double kI;
  private double kD;
  private PIDController pidController;
  private double pidOutput;
  private double initAngle;

  /**
   * LEFT IS POSITIVE
   * @param driveBase
   * @param gyro
   * @param angle
   */
  public TurnWithGyroClosedLoop(DriveBaseSubsystem driveBase, GyroSubsystem gyroSubsystem, double target, double kP, double kI, double kD) {
    this.driveBase = driveBase;
    this.gyroSubsystem = gyroSubsystem;
    this.target = target;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  @Override
  public void initialize() {
    driveBase.coast();
    initAngle = gyroSubsystem.getGyroAngle();
    pidController = new PIDController(kP, kI, kD);
    pidController.setSetpoint(initAngle + target);
    pidController.setTolerance(.5); //0.5 -> 1.5
  } 

  @Override
  public void execute() {
    pidOutput = pidController.calculate(gyroSubsystem.getGyroAngle());
    driveBase.setLeftPower(-pidOutput);
    driveBase.setRightPower(pidOutput);
    SmartDashboard.putNumber("gyro turn error", pidController.getPositionError());
    SmartDashboard.putNumber("robot turned", gyroSubsystem.getGyroAngle() - initAngle);
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.stop();
    driveBase.brake();
    
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
