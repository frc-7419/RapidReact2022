package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.servo.ServoSubsystem;

public class TurnLimelightWithServoToTyOpenLoop extends CommandBase {

  private ServoSubsystem servoSubsystem;
  private LimelightSubsystem limelightSubsystem;

  private double initAngle;
  private double ty;
  
  public TurnLimelightWithServoToTyOpenLoop(ServoSubsystem servoSubsystem, LimelightSubsystem limelightSubsystem, double initAngle) {
    this.servoSubsystem = servoSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.initAngle = initAngle;
    addRequirements(servoSubsystem);
  }

  @Override
  public void initialize() {
    servoSubsystem.setAngle(initAngle);
  }

  @Override
  public void execute() {
    ty = limelightSubsystem.getTy();
    if (limelightSubsystem.getTv() == 1) {
      servoSubsystem.setAngle(servoSubsystem.getAngle() + ty);
    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
 