package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private ProfiledPIDController pidController;
  private double pidOutput;
  private double negative;
  private double initAngle;

  /**
   * LEFT IS POSITIVE
   * @param driveBase
   * @param gyro
   * @param angle
   */
  public TurnWithGyroClosedLoop(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
  }

  @Override
  public void initialize() {
    if (target > 0){negative = 1;}
    else {negative = -1;}
    initAngle = gyroSubsystem.getGyroAngle();

    driveBaseSubsystem.coast();

    double kP = SmartDashboard.getNumber("kp", PIDConstants.GyrokP);
    double kI = SmartDashboard.getNumber("ki", PIDConstants.GyrokI);
    double kD = SmartDashboard.getNumber("kd", PIDConstants.GyrokD);
    double kMaxVelocity = SmartDashboard.getNumber("kMaxVelocity", 5);
    double kMaxAcceleration = SmartDashboard.getNumber("kMaxAcc", 5);
    target = SmartDashboard.getNumber("setpoint", 180);

    pidController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration));
    pidController.setGoal(initAngle + target);
    pidController.setTolerance(0.5);
  } 

  @Override
  public void execute() {
    SmartDashboard.putNumber("gyro turn error", pidController.getPositionError());
    SmartDashboard.putBoolean("at setpoint", pidController.atSetpoint());
    pidOutput = pidController.calculate(gyroSubsystem.getGyroAngle());
    driveBaseSubsystem.setLeftPower(negative * -pidOutput);
    driveBaseSubsystem.setRightPower(negative * pidOutput);
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.stop();
    driveBaseSubsystem.brake();
    // Timer.delay(1);
    SmartDashboard.putNumber("robot turned", gyroSubsystem.getGyroAngle() - initAngle);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
