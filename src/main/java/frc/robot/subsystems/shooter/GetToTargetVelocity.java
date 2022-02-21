package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;

public class GetToTargetVelocity extends CommandBase {

  private ShooterSubsystem shooterSubsystem;

  private double bKp;
  private double bKi;
  private double tKp;
  private double tKi;
  private double bKf;
  private double tKf;

  private double topTargetRawVelocity;
  private double bottomTargetRawVelocity;

  private double topTargetRawAcceleration;
  private double bottomTargetRawAcceleration;

  public GetToTargetVelocity(ShooterSubsystem shooterSubsystem, double topTargetRawVelocity, double bottomTargetRawVelocity) {
    this.shooterSubsystem = shooterSubsystem;
    this.topTargetRawVelocity = topTargetRawVelocity;
    this.bottomTargetRawVelocity = bottomTargetRawVelocity;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Shooter Running", false);

    topTargetRawVelocity = SmartDashboard.getNumber("tTargetRV", topTargetRawVelocity);
    bottomTargetRawVelocity = SmartDashboard.getNumber("bTargetRV", bottomTargetRawVelocity);
    
    bKp = SmartDashboard.getNumber("bKp", PIDConstants.BottomShooterkP);
    bKi = SmartDashboard.getNumber("bKi", PIDConstants.BottomShooterkI);

    tKp = SmartDashboard.getNumber("tKp", PIDConstants.TopShooterkP);
    tKi = SmartDashboard.getNumber("tKi", PIDConstants.TopShooterkI);

    // shooterSubsystem.setkF(shooterSubsystem.computekF(topTargetRPM));

    tKf = shooterSubsystem.computeTopkF(topTargetRawVelocity);
    bKf = shooterSubsystem.computeBottomkF(bottomTargetRawVelocity);

    shooterSubsystem.setTopPIDF(bKp, bKi, 0, tKf);
    shooterSubsystem.setBottomPIDF(tKp, tKi, 0, bKf);
    
    // instance var setter method for ShooterSubsystem
    // shooterSubsystem.setTopTargetRawVelocity(shooterSubsystem.rpmToRawSensorVelocity(topTargetRPM, ticksPerRotation));
    // shooterSubsystem.setBottomTargetRawVelocity(shooterSubsystem.rpmToRawSensorVelocity(topTargetRPM, ticksPerRotation));

    shooterSubsystem.setTopTargetRawVelocity(topTargetRawVelocity);
    shooterSubsystem.setBottomTargetRawVelocity(bottomTargetRawVelocity);
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("Shooter Running", true);

    // update PIF values from SD while running
    bKp = SmartDashboard.getNumber("bKp", PIDConstants.BottomShooterkP);
    bKi = SmartDashboard.getNumber("bKi", PIDConstants.BottomShooterkI);

    tKp = SmartDashboard.getNumber("tKp", PIDConstants.TopShooterkP);
    tKi = SmartDashboard.getNumber("tKi", PIDConstants.TopShooterkI);

    tKf = shooterSubsystem.computeTopkF(topTargetRawVelocity);
    bKf = shooterSubsystem.computeBottomkF(bottomTargetRawVelocity);

    shooterSubsystem.setTopPIDF(bKp, bKi, 0, tKf);
    shooterSubsystem.setBottomPIDF(tKp, tKi, 0, bKf);

    topTargetRawVelocity = SmartDashboard.getNumber("tTargetRPM", topTargetRawVelocity);
    bottomTargetRawVelocity = SmartDashboard.getNumber("bTargetRPM", bottomTargetRawVelocity);

    // double topTargetVelocity = topTargetRPM * ticksPerRotation * (1/600);
    // double bottomTargetVelocity = bottomTargetRPM * ticksPerRotation * (1/600);
    
    // SmartDashboard.putNumber("tTargetRV", topTargetRawVelocity);
    // SmartDashboard.putNumber("bTargetRV", bottomTargetRawVelocity);
    
    shooterSubsystem.getTopTalon().set(ControlMode.Velocity, topTargetRawVelocity);
    shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, bottomTargetRawVelocity);

    SmartDashboard.putBoolean("Top On Target", shooterSubsystem.topOnTarget());
    SmartDashboard.putBoolean("Bottom on Target", shooterSubsystem.bottomOnTarget());
    SmartDashboard.putBoolean("Both on Target", shooterSubsystem.bothOnTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.off();
    SmartDashboard.putBoolean("Shooter Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
