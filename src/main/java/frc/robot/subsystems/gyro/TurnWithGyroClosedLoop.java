package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class TurnWithGyroClosedLoop extends CommandBase {
  
  private DriveBaseSubsystem driveBase;
  private GyroSubsystem ahrs;
  private double target;
  private double kP;
  private double kI;
  private double kD;
  private PIDController pidController;
  private double pidOutput;
  private double negative;
  private double initAngle;

  /**
   * LEFT IS POSITIVE
   * @param driveBase
   * @param gyro
   * @param angle
   */
  public TurnWithGyroClosedLoop(DriveBaseSubsystem driveBase, GyroSubsystem ahrs, double target) {
    this.driveBase = driveBase;
    this.ahrs = ahrs;
    this.target = target;
  }

  @Override
  public void initialize() {
    if(target > 0){negative = 1;}
    else{negative = -1;}
    initAngle = ahrs.getGyroAngle();
    // SmartDashboard.putNumber("kp", PIDConstants.GyrokP);
    // SmartDashboard.putNumber("kd", PIDConstants.GyrokD);
    double kp = SmartDashboard.getNumber("kp", PIDConstants.GyrokP);
    double ki = SmartDashboard.getNumber("kp", PIDConstants.GyrokI);
    double kd = SmartDashboard.getNumber("kd", PIDConstants.GyrokD);
    pidController = new PIDController(kp, ki, kd);
    // pidController = new PIDController(PIDConstants.GyrokP, PIDConstants.GyrokI, PIDConstants.GyrokD);
    pidController.setSetpoint(initAngle + target);
    pidController.setTolerance(0.1); 
  } 

  @Override
  public void execute() {
    SmartDashboard.putString("command status", "turn w gyro");
    SmartDashboard.putNumber("gyro turn error", pidController.getPositionError());
    pidOutput = pidController.calculate(ahrs.getGyroAngle());
    driveBase.setLeftPower(negative * -pidOutput);
    driveBase.setRightPower(negative * pidOutput);
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.stop();
    driveBase.brake();
    // Timer.delay(1);
    SmartDashboard.putNumber("robot turned", ahrs.getGyroAngle() - initAngle);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
