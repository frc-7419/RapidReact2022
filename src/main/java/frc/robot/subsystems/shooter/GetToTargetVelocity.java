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
  private double topTargetVelocity;
  private double bottomTargetVelocity;

  private double ticksPerRotation = 2048;

  public GetToTargetVelocity(ShooterSubsystem shooterSubsystem, double topTargetVelocity, double bottomTargetVelocity) {
    this.shooterSubsystem = shooterSubsystem;
    this.topTargetVelocity = topTargetVelocity;
    this.bottomTargetVelocity = bottomTargetVelocity;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("GTV Running", false);

    topTargetVelocity = SmartDashboard.getNumber("topTargetVelocity", topTargetVelocity);
    bottomTargetVelocity = SmartDashboard.getNumber("bottomTargeVelocity", bottomTargetVelocity);
    
    // shooterSubsystem.setkF(shooterSubsystem.computekF(topTargetRPM));
    
    kP = SmartDashboard.getNumber("shooterKp", PIDConstants.ShooterkP);
    kI = SmartDashboard.getNumber("shooterKi", PIDConstants.ShooterkI);

    bottomKf = SmartDashboard.getNumber("tKf", PIDConstants.ShooterkF);
    topKf = SmartDashboard.getNumber("bKf", PIDConstants.ShooterkF);

    shooterSubsystem.setTopPIDF(kP, kI, 0, topKf);
    shooterSubsystem.setBottomPIDF(kP, kI, 0, bottomKf);
    
    // instance var setter method for ShooterSubsystem
    shooterSubsystem.setTopTargetVelocity(topTargetVelocity);
    shooterSubsystem.setBottomTargetVelocity(bottomTargetVelocity);
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

    topTargetVelocity = SmartDashboard.getNumber("topTargetVelocity", topTargetVelocity);
    bottomTargetVelocity= SmartDashboard.getNumber("bottomTargetVelocity", bottomTargetVelocity);

    // shooterSubsystem.getTopTalon().set(ControlMode.Velocity, UnitConversions.rpmToRawSensorVelocity(topTargetRPM, 2048));
    // shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, UnitConversions.rpmToRawSensorVelocity(bottomTargetRPM, 2048));

    double bottomTargetRPM = UnitConversions.rpmToRawSensorVelocity(bottomTargetVelocity, ticksPerRotation);
    double topTargetRPM = UnitConversions.rpmToRawSensorVelocity(topTargetVelocity, ticksPerRotation);

    SmartDashboard.putNumber("topTargetRPM", topTargetRPM);
    SmartDashboard.putNumber("bottomTargetRPM", bottomTargetRPM);
    
    shooterSubsystem.getTopTalon().set(ControlMode.Velocity, topTargetVelocity);
    shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, bottomTargetVelocity);

    // SmartDashboard.putNumber("top error", shooterSubsystem.getTopTalon().getClosedLoopError());
    // SmartDashboard.putNumber("bottom error", shooterSubsystem.getBottomTalon().getClosedLoopError());

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
