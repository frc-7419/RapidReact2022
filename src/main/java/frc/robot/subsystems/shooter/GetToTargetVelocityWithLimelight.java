package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.InterpolatedTreeMap;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class GetToTargetVelocityWithLimelight extends CommandBase {

  private ShooterSubsystem shooterSubsystem;
  private LimelightSubsystem limelightSubsystem;

  private InterpolatedTreeMap topShooterReferencePoints;
  private InterpolatedTreeMap bottomShooterReferencePoints;

  private double kP;
  private double kI;
  private double tkF;
  private double bkF;

  private double initialVelocity;

  // private double targetRPM;
  private double topTargetRawVelocity;
  private double bottomTargetRawVelocity;

  
  public GetToTargetVelocityWithLimelight(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(shooterSubsystem);

    topShooterReferencePoints = new InterpolatedTreeMap();
    bottomShooterReferencePoints = new InterpolatedTreeMap();

    // top example:
    topShooterReferencePoints.put(1.36, 28.0);

    // bottom example:
    bottomShooterReferencePoints.put(1.36, 28.0);
  }

  @Override
  public void initialize() {
    topTargetRawVelocity = topShooterReferencePoints.get(limelightSubsystem.getDistance());
    bottomTargetRawVelocity = bottomShooterReferencePoints.get(limelightSubsystem.getDistance());

    shooterSubsystem.setTopTargetRawVelocity(topTargetRawVelocity);
    shooterSubsystem.setBottomTargetRawVelocity(bottomTargetRawVelocity);

    shooterSubsystem.setTopPIDF(kP, kI, 0, shooterSubsystem.computeTopkF(topTargetRawVelocity));
    shooterSubsystem.setBottomPIDF(kP, kI, 0, shooterSubsystem.computeBottomkF(bottomTargetRawVelocity));
  }

  @Override
  public void execute() {
    shooterSubsystem.getTopTalon().set(ControlMode.Velocity, topTargetRawVelocity);
    shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, bottomTargetRawVelocity);

    // SmartDashboard.putBoolean("Top On Target", shooterSubsystem.topOnTarget());
    // SmartDashboard.putBoolean("Bottom on Target", shooterSubsystem.bottomOnTarget());
    // SmartDashboard.putBoolean("Both on Target", shooterSubsystem.bothOnTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
