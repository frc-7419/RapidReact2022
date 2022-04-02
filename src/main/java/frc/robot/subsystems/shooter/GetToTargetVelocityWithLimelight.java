package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.InterpolatedTreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class GetToTargetVelocityWithLimelight extends CommandBase {

  private ShooterSubsystem shooterSubsystem;
  private LimelightSubsystem limelightSubsystem;

  private InterpolatedTreeMap topShooterReferencePoints;
  private InterpolatedTreeMap bottomShooterReferencePoints;

  private double bKp;
  private double bKi;
  private double tKp;
  private double tKi;

  private double topTargetRawVelocity;
  private double bottomTargetRawVelocity;

  public GetToTargetVelocityWithLimelight(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(shooterSubsystem);

    topShooterReferencePoints = new InterpolatedTreeMap();
    bottomShooterReferencePoints = new InterpolatedTreeMap();

    // config reference points from constants file
    shooterSubsystem.configInterpolatedTreeMapReferencePoints(Constants.kDistanceToTopShooterRawVelocity, topShooterReferencePoints);
    shooterSubsystem.configInterpolatedTreeMapReferencePoints(Constants.kDistanceToBottomShooterRawVelocity, bottomShooterReferencePoints);
  }

  @Override
  public void initialize() {
    topTargetRawVelocity = topShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();
    bottomTargetRawVelocity = bottomShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();

    shooterSubsystem.setTopTargetRawVelocity(topTargetRawVelocity);
    shooterSubsystem.setBottomTargetRawVelocity(bottomTargetRawVelocity);

    shooterSubsystem.setTopPIDF(tKp, tKi, 0, shooterSubsystem.computeTopkF(topTargetRawVelocity));
    shooterSubsystem.setBottomPIDF(bKp, bKi, 0, shooterSubsystem.computeBottomkF(bottomTargetRawVelocity));
  }

  @Override
  public void execute() {
    topTargetRawVelocity = topShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();
    bottomTargetRawVelocity = bottomShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();

    shooterSubsystem.setTopTargetRawVelocity(topTargetRawVelocity);
    shooterSubsystem.setBottomTargetRawVelocity(bottomTargetRawVelocity);

    shooterSubsystem.setTopPIDF(tKp, tKi, 0, shooterSubsystem.computeTopkF(topTargetRawVelocity));
    shooterSubsystem.setBottomPIDF(bKp, bKi, 0, shooterSubsystem.computeBottomkF(bottomTargetRawVelocity));
    
    shooterSubsystem.getTopTalon().set(ControlMode.Velocity, topTargetRawVelocity);
    shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, bottomTargetRawVelocity);

    // SmartDashboard.putBoolean("Top On Target", shooterSubsystem.topOnTarget());
    // SmartDashboard.putBoolean("Bottom on Target", shooterSubsystem.bottomOnTarget());
    // SmartDashboard.putBoolean("Both on Target", shooterSubsystem.bothOnTarget());
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Shooter Running", false);
    shooterSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
