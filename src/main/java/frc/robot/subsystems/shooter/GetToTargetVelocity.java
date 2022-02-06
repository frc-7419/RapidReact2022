package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.math.UnitConversions;

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
  private double topTargetRPM;
  private double bottomTargetRPM;

  private double ticksPerRotation = 2048;

  public GetToTargetVelocity(ShooterSubsystem shooterSubsystem, double topTargetRPM, double bottomTargetRPM) {
    this.shooterSubsystem = shooterSubsystem;
    this.topTargetRPM = topTargetRPM;
    this.bottomTargetRPM = bottomTargetRPM;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("GTV Running", false);

    topTargetRPM = SmartDashboard.getNumber("tTargetRPM", topTargetRPM);
    bottomTargetRPM = SmartDashboard.getNumber("bTargetRPM", bottomTargetRPM);
    
    // shooterSubsystem.setkF(shooterSubsystem.computekF(topTargetRPM));
    
    kP = SmartDashboard.getNumber("shooterKp", PIDConstants.ShooterkP);
    kI = SmartDashboard.getNumber("shooterKi", PIDConstants.ShooterkI);

    bottomKf = SmartDashboard.getNumber("tKf", PIDConstants.ShooterkF);
    topKf = SmartDashboard.getNumber("bKf", PIDConstants.ShooterkF);

    shooterSubsystem.setTopPIDF(kP, kI, 0, topKf);
    shooterSubsystem.setBottomPIDF(kP, kI, 0, bottomKf);
    
    // instance var setter method for ShooterSubsystem
    shooterSubsystem.setTopTargetVelocity(shooterSubsystem.rpmToRawSensorVelocity(topTargetRPM, ticksPerRotation));
    shooterSubsystem.setBottomTargetVelocity(shooterSubsystem.rpmToRawSensorVelocity(topTargetRPM, ticksPerRotation));
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("GTV Running", true);

    // update PIF values from SD while running
    kP = SmartDashboard.getNumber("shooterKp", PIDConstants.ShooterkP);
    kI = SmartDashboard.getNumber("shooterKi", PIDConstants.ShooterkI);

    bottomKf = SmartDashboard.getNumber("tKf", PIDConstants.ShooterkF);
    topKf = SmartDashboard.getNumber("bKf", PIDConstants.ShooterkF);

    shooterSubsystem.setTopPIDF(kP, kI, 0, topKf);
    shooterSubsystem.setBottomPIDF(kP, kI, 0, bottomKf);

    topTargetRPM = SmartDashboard.getNumber("tTargetRPM", topTargetRPM);
    bottomTargetRPM = SmartDashboard.getNumber("bTargetRPM", bottomTargetRPM);

    double bottomTargetVelocity = bottomTargetRPM * ticksPerRotation * (1/600);
    double topTargetVelocity = topTargetRPM * ticksPerRotation * (1/600);

    SmartDashboard.putNumber("topTargetRPM", topTargetRPM);
    SmartDashboard.putNumber("bottomTargetRPM", bottomTargetRPM);
    
    shooterSubsystem.getTopTalon().set(ControlMode.Velocity, topTargetVelocity);
    shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, bottomTargetVelocity);

    SmartDashboard.putBoolean("Top On Target", shooterSubsystem.topOnTarget());
    SmartDashboard.putBoolean("Bottom on Target", shooterSubsystem.bottomOnTarget());
    SmartDashboard.putBoolean("Both on Target", shooterSubsystem.bothOnTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.off();
    SmartDashboard.putBoolean("GTV Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
