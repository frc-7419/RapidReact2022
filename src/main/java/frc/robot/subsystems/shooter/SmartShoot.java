// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.team7419.InterpolatedTreeMap;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class SmartShoot extends CommandBase {
  /** Creates a new SmartShoot. */
  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;
  private LoaderSubsystem loaderSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private BeamBreakSubsystem beamBreakSubsystem;

  private InterpolatedTreeMap topShooterReferencePoints;
  private InterpolatedTreeMap bottomShooterReferencePoints;

  private double bKp = 0;
  private double tKp = 0;

  private double topTargetVelocity;
  private double bottomTargetVelocity;
  private double velocityThreshold = 10;
  public SmartShoot(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, LoaderSubsystem loaderSubsystem,
      LimelightSubsystem limelightSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.loaderSubsystem = loaderSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.beamBreakSubsystem = beamBreakSubsystem;
    addRequirements(shooterSubsystem, feederSubsystem, loaderSubsystem, limelightSubsystem, beamBreakSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    topShooterReferencePoints = new InterpolatedTreeMap();
    bottomShooterReferencePoints = new InterpolatedTreeMap();

    // config reference points from constants file
    shooterSubsystem.configInterpolatedTreeMapReferencePoints(Constants.kDistanceToTopShooterVelocity,
        topShooterReferencePoints);
    shooterSubsystem.configInterpolatedTreeMapReferencePoints(Constants.kDistanceToBottomShooterVelocity,
        bottomShooterReferencePoints);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    topTargetVelocity = topShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();
    bottomTargetVelocity = bottomShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();

    shooterSubsystem.setTopPIDF(tKp, 0, 0, 0);
    shooterSubsystem.setBottomPIDF(bKp, 0, 0, 0);

    shooterSubsystem.setTopClosedLoopVelocity(topTargetVelocity);
    shooterSubsystem.setBottomClosedLoopVelocity(bottomTargetVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if beam break is not broken, run shooter to speed using limelight
    // once shooter is up to speed, run feeder and loader
    // if break is broken, run feeder and loader in reverse
    if (beamBreakSubsystem.getBeamBreakActivated()) {
      if (Math.abs(shooterSubsystem.getCurrentTopVelocity()-topTargetVelocity)>velocityThreshold || Math.abs(shooterSubsystem.getCurrentBottomVelocity()-bottomTargetVelocity)>velocityThreshold) {
        topTargetVelocity = topShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();
        bottomTargetVelocity = bottomShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();

        shooterSubsystem.setTopPIDF(tKp, 0, 0, 0);
        shooterSubsystem.setBottomPIDF(bKp, 0, 0, 0);

        shooterSubsystem.setTopClosedLoopVelocity(topTargetVelocity);
        shooterSubsystem.setBottomClosedLoopVelocity(bottomTargetVelocity);
        feederSubsystem.setPower(-0.2);
      } else {
        shooterSubsystem.setTopClosedLoopVelocity(topTargetVelocity);
        shooterSubsystem.setBottomClosedLoopVelocity(bottomTargetVelocity);
        feederSubsystem.setPower(1);
        loaderSubsystem.setPower(1);
      }
    } else {
      if (!shooterSubsystem.bothOnTarget()) {
        feederSubsystem.setPower(-1);
        loaderSubsystem.setPower(-1);
      } else {
        feederSubsystem.setPower(1);
        loaderSubsystem.setPower(1);
      }
    }
  }

  // Called once the command ends or is interrupted.
  public void runShooterWithLimelight() {
    topTargetVelocity = topShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();
    bottomTargetVelocity = bottomShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();

    shooterSubsystem.setTopPIDF(tKp, 0, 0, 0);
    shooterSubsystem.setBottomPIDF(bKp, 0, 0, 0);

    shooterSubsystem.setTopClosedLoopVelocity(topTargetVelocity);
    shooterSubsystem.setBottomClosedLoopVelocity(bottomTargetVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
