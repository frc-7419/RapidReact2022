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

  private double ticksPerRotation = 2048;

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
    
    kP = SmartDashboard.getNumber("shooterKp", PIDConstants.ShooterkP);
    kI = SmartDashboard.getNumber("shooterKi", PIDConstants.ShooterkI);

    // shooterSubsystem.setkF(shooterSubsystem.computekF(topTargetRPM));

    bottomKf = SmartDashboard.getNumber("bKf", PIDConstants.ShooterkF);
    topKf = SmartDashboard.getNumber("tKf", PIDConstants.ShooterkF);

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
    kP = SmartDashboard.getNumber("shooterKp", PIDConstants.ShooterkP);
    kI = SmartDashboard.getNumber("shooterKi", PIDConstants.ShooterkI);

    bottomKf = SmartDashboard.getNumber("bKf", PIDConstants.ShooterkF);
    topKf = SmartDashboard.getNumber("tKf", PIDConstants.ShooterkF);

    shooterSubsystem.setTopPIDF(kP, kI, 0, topKf);
    shooterSubsystem.setBottomPIDF(kP, kI, 0, bottomKf);

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
