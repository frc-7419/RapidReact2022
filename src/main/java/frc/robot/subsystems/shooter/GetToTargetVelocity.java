package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;

public class GetToTargetVelocity extends CommandBase {

  private ShooterSubsystem shooterSubsystem;

  private double kP;
  private double kI;
  // private double kF;
  private double bottomKf;
  private double topKf;

  // private double targetRPM;
  private double topTargetRawVelocity;
  private double bottomTargetRawVelocity;

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
    
    kP = SmartDashboard.getNumber("ShooterKp", 0);
    kI = SmartDashboard.getNumber("ShooterKi", 0);

    // shooterSubsystem.setkF(shooterSubsystem.computekF(topTargetRPM));

    bottomKf = SmartDashboard.getNumber("bKf", 0.05);
    topKf = SmartDashboard.getNumber("tKf", 0.05);

    shooterSubsystem.setTopPIDF(kP, kI, 0, topKf);
    shooterSubsystem.setBottomPIDF(kP, kI, 0, bottomKf);
    
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
    kP = SmartDashboard.getNumber("ShooterKp", 0);
    kI = SmartDashboard.getNumber("ShooterKi", 0);

    bottomKf = SmartDashboard.getNumber("bKf", 0.05);
    topKf = SmartDashboard.getNumber("tKf", 0.05);

    shooterSubsystem.setTopPIDF(kP, kI, 0, topKf);
    shooterSubsystem.setBottomPIDF(kP, kI, 0, bottomKf);

    topTargetRawVelocity = SmartDashboard.getNumber("tTargetRV", topTargetRawVelocity);
    bottomTargetRawVelocity = SmartDashboard.getNumber("bTargetRV", bottomTargetRawVelocity);

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
