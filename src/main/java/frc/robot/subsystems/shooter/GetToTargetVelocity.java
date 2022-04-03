package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;

public class GetToTargetVelocity extends CommandBase {

  private ShooterSubsystem shooterSubsystem;

  private double bKp;
  private double bKi;
  private double tKp;
  private double tKi;

  private double topTargetVelocity;
  private double bottomTargetVelocity;

  private double topTargetRawAcceleration;
  private double bottomTargetRawAcceleration;

  public GetToTargetVelocity(ShooterSubsystem shooterSubsystem, double topTargetVelocity, double bottomTargetVelocity) {
    this.shooterSubsystem = shooterSubsystem;
    this.topTargetVelocity = topTargetVelocity;
    this.bottomTargetVelocity = bottomTargetVelocity;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Shooter Running", false);
    
    // bKp = SmartDashboard.getNumber("bKp", PIDConstants.BottomShooterkP);
    // bKi = SmartDashboard.getNumber("bKi", PIDConstants.BottomShooterkI);

    // tKp = SmartDashboard.getNumber("tKp", PIDConstants.TopShooterkP);
    // tKi = SmartDashboard.getNumber("tKi", PIDConstants.TopShooterkI);

    shooterSubsystem.setTopPIDF(bKp, bKi, 0, 0);
    shooterSubsystem.setBottomPIDF(tKp, tKi, 0, 0);
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("Shooter Running", true);

    bKp = SmartDashboard.getNumber("bKp", PIDConstants.BottomShooterkP);
    // bKi = SmartDashboard.getNumber("bKi", PIDConstants.BottomShooterkI);

    tKp = SmartDashboard.getNumber("tKp", PIDConstants.TopShooterkP);
    // tKi = SmartDashboard.getNumber("tKi", PIDConstants.TopShooterkI);

    topTargetVelocity = SmartDashboard.getNumber("tTargetVelocity", topTargetVelocity);
    bottomTargetVelocity = SmartDashboard.getNumber("bTargetVelocity", bottomTargetVelocity);

    shooterSubsystem.setTopPIDF(0, 0, 0, 0);
    shooterSubsystem.setBottomPIDF(0, 0, 0, 0);

    shooterSubsystem.setTopClosedLoopVelocity(topTargetVelocity);
    shooterSubsystem.setBottomClosedLoopVelocity(bottomTargetVelocity);

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
