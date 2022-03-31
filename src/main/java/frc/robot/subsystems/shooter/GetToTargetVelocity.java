package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;

public class GetToTargetVelocity extends CommandBase {

  private ShooterSubsystem shooterSubsystem;

  private double tKp;
  private double bKp;

  private double tKi;
  private double bKi;

  private double bKf;
  private double tKf;

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
    
    tKp = SmartDashboard.getNumber("tKp", PIDConstants.ShooterTopKp);
    bKp = SmartDashboard.getNumber("bKp", PIDConstants.ShooterBottomKp);

    tKi = SmartDashboard.getNumber("tKi", PIDConstants.ShooterTopKi);
    bKi = SmartDashboard.getNumber("bKi", PIDConstants.ShooterBottomKi);

    bKf = SmartDashboard.getNumber("bKf", 0);
    tKf = SmartDashboard.getNumber("tKf", 0);

    shooterSubsystem.setTopPIDF(tKp, tKi, 0, tKf);
    shooterSubsystem.setBottomPIDF(bKp, bKi, 0, bKf);

    shooterSubsystem.setTopTargetRawVelocity(topTargetRawVelocity);
    shooterSubsystem.setBottomTargetRawVelocity(bottomTargetRawVelocity);
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("Shooter Running", true);

    topTargetRawVelocity = SmartDashboard.getNumber("tTargetRV", topTargetRawVelocity);
    bottomTargetRawVelocity = SmartDashboard.getNumber("bTargetRV", bottomTargetRawVelocity);
    
    tKp = SmartDashboard.getNumber("tKp", PIDConstants.ShooterTopKp);
    bKp = SmartDashboard.getNumber("bKp", PIDConstants.ShooterBottomKp);

    tKi = SmartDashboard.getNumber("tKi", PIDConstants.ShooterTopKi);
    bKi = SmartDashboard.getNumber("bKi", PIDConstants.ShooterBottomKi);

    bKf = SmartDashboard.getNumber("bKf", 0);
    tKf = SmartDashboard.getNumber("tKf", 0);

    shooterSubsystem.setTopPIDF(tKp, tKi, 0, tKf);
    shooterSubsystem.setBottomPIDF(bKp, bKi, 0, bKf);

    shooterSubsystem.setTopTargetRawVelocity(topTargetRawVelocity);
    shooterSubsystem.setBottomTargetRawVelocity(bottomTargetRawVelocity);
    
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
