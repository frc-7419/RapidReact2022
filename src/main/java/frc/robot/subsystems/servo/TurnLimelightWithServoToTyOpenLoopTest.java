package frc.robot.subsystems.servo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class TurnLimelightWithServoToTyOpenLoopTest extends CommandBase {
  
  private ServoSubsystem servoSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private PIDController pidController;

  private double ty;
  private double tv;
  private double initAngle = 70;
  
  private double kP;
  private double kI;
  private double kD;

  private double pidOutput;
  
  public TurnLimelightWithServoToTyOpenLoopTest(ServoSubsystem servoSubsystem, LimelightSubsystem limelightSubsystem) {
    this.servoSubsystem = servoSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(servoSubsystem);
  }

  @Override
  public void initialize() {
    // kP = .016;
    // kI = 0;
    // kD = 1; 
    // pidController = new PIDController(kP, kI, kD);
    // pidController.setSetpoint(0);
    // pidController.setTolerance(2);
    servoSubsystem.setAngle(initAngle);
;  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ty = limelightSubsystem.getTy();
    tv = limelightSubsystem.getTv();

    if (tv == 1.0 && (ty > .5 || ty < .5)) {
      // pidOutput = pidController.calculate(ty);
      if (ty > .5) {
        servoSubsystem.setAngle(servoSubsystem.getAngle() + .5);
      }
      else {
        servoSubsystem.setAngle(servoSubsystem.getAngle() - .5);
      }
    }

    SmartDashboard.putNumber("current angle", servoSubsystem.getAngle());
    SmartDashboard.putNumber("ty +- mounting", ty + initAngle);

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
