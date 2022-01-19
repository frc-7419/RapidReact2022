package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.servo.ServoSubsystem;

public class TurnLimelightWithServoToTy extends CommandBase {

  private ServoSubsystem servoSubsystem;
  private LimelightSubsystem limelightSubsystem;

  private double initAngle;
  private double tx;
  private double ty;
  
  public TurnLimelightWithServoToTy(ServoSubsystem servoSubsystem, LimelightSubsystem limelightSubsystem) {
    this.servoSubsystem = servoSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(servoSubsystem);
  }

  @Override
  public void initialize() {
    initAngle = servoSubsystem.getAngle();
  }

  @Override
  public void execute() {
    tx = limelightSubsystem.getTx();
    ty = limelightSubsystem.getTy();

    servoSubsystem.setAngle(ty);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
 