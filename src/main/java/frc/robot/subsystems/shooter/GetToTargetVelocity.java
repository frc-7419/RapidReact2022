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
  private double kF;

  private double targetRPM;

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
    // SmartDashboard.putString("shooter", "ramping up");

    // shooterSubsystem.configShooterOutputs();

    // topTargetRPM = SmartDashboard.getNumber("top targetRPM", topTargetRPM);
    // bottomTargetRPM = SmartDashboard.getNumber("bottom targetRPM", bottomTargetRPM);
    
    // shooterSubsystem.setkF(shooterSubsystem.computekF(topTargetRPM));
    
    // kP = SmartDashboard.getNumber("shooterKp", PIDConstants.ShooterkP);
    // kI = SmartDashboard.getNumber("shooterKi", PIDConstants.ShooterkI);

    kP = 0;
    kI = 0;
    kF = 0.05;

    shooterSubsystem.setTopPIDF(kP, kI, 0, kF);
    shooterSubsystem.setBottomPIDF(kP, kI, 0, kF);
    
    shooterSubsystem.setTopTargetVelocity(15000);
    shooterSubsystem.setBottomTargetVelocity(15000);
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("GTV Running", true);

    // shooterSubsystem.getTopTalon().set(ControlMode.Velocity, UnitConversions.rpmToRawSensorVelocity(topTargetRPM, 2048));
    // shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, UnitConversions.rpmToRawSensorVelocity(bottomTargetRPM, 2048));

    // targetRPM = UnitConversions.rpmToRawSensorVelocity(2500, ticksPerRotation);
    shooterSubsystem.getTopTalon().set(ControlMode.Velocity, 15000);
    shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, 15000);

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
