package frc.robot.subsystems.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.servo.ServoSubsystem;

public class TurnLimelightWithServoToTyClosedLoop extends CommandBase {
  
  private ServoSubsystem servoSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private PIDController pidController;

  private double ty;
  private double tv;
  private double servoAngle;
  
  private double kP;
  private double kI;
  private double kD;

  private double pidOutput;
  
  public TurnLimelightWithServoToTyClosedLoop(ServoSubsystem servoSubsystem, LimelightSubsystem limelightSubsystem) {
    this.servoSubsystem = servoSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(servoSubsystem);
  }

  @Override
  public void initialize() {
    kP = .016; // gets P coefficient from dashboard
    kI = 0;
    kD = 1; 
    pidController = new PIDController(kP, kI, kD);
    pidController.setSetpoint(0);
    pidController.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ty = limelightSubsystem.getTy();
    tv = limelightSubsystem.getTv();

    if (tv == 1.0 && (ty > 0.05 || ty < 0.05)) {
      pidOutput = pidController.calculate(ty);
      servoAngle = servoSubsystem.getAngle();
      servoSubsystem.setAngle(servoAngle + pidOutput);
      SmartDashboard.putNumber("current angle", servoSubsystem.getAngle());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
