package frc.robot.subsystems.servo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class TurnLimelightWithServoToTyClosedLoop extends CommandBase {
  
  private ServoSubsystem servoSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private PIDController pidController;

  private double kP = PIDConstants.limelightServokP;
  private double kI = PIDConstants.limelightServokI;
  private double kD = PIDConstants.limelightServokD;

  private double ty;
  private double tv;
  private double initAngle = 70;

  private int directionalCoefficient;
  private double pidOutput;
  
  public TurnLimelightWithServoToTyClosedLoop(ServoSubsystem servoSubsystem, LimelightSubsystem limelightSubsystem) {
    this.servoSubsystem = servoSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(servoSubsystem);
  }

  @Override
  public void initialize() {

    pidController = new PIDController(kP, kI, kD);

    servoSubsystem.setAngle(initAngle);

    pidController.setSetpoint(initAngle + limelightSubsystem.getTy());
    pidController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ty = limelightSubsystem.getTy();
    tv = limelightSubsystem.getTv();

    if (ty < 0)
        directionalCoefficient = 1;
    else
        directionalCoefficient = -1;
    
    if (tv == 1.0) {
        pidOutput = pidController.calculate(ty);
        servoSubsystem.setAngle(servoSubsystem.getAngle() + (directionalCoefficient*pidOutput));
    }

    SmartDashboard.putNumber("current angle: ", servoSubsystem.getAngle());
    SmartDashboard.putNumber("ty + initial: ", (ty + initAngle));
    SmartDashboard.putNumber("servo pid output: ", pidOutput);
    SmartDashboard.putNumber("closed loop error: ", pidController.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return pidController.atSetpoint();
    return false;
  }
}
