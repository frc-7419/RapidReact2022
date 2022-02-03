package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class GetToTargetVelocity extends CommandBase {

  private ShooterSubsystem shooterSubsystem;
  private LimelightSubsystem limelightSubsystem;

  private double kP;
  private double kI;
  private double kF;

  private double initialVelocity;

  // private double targetRPM;
  private double topTargetRPM;
  private double bottomTargetRPM;

  
  public GetToTargetVelocity(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("shooter", "ramping up");
    
    topTargetRPM = SmartDashboard.getNumber("targetRPM", 1000);
    bottomTargetRPM = SmartDashboard.getNumber("targetRPM", 1000);
    
    shooterSubsystem.setkF(shooterSubsystem.computekF(topTargetRPM));
    
    kP = SmartDashboard.getNumber("shooterKp", PIDConstants.ShooterkP);
    kI = SmartDashboard.getNumber("shooterKi", PIDConstants.ShooterkI);

    shooterSubsystem.setPIDF(kP, kI, 0, shooterSubsystem.getkF());
    
    shooterSubsystem.setTopTargetRawVelocity(UnitConversions.rpmToRawSensorVelocity(topTargetRPM, 2048));
    shooterSubsystem.setBottomTargetRawVelocity(UnitConversions.rpmToRawSensorVelocity(bottomTargetRPM, 2048));
    
  }

  @Override
  public void execute() {
    shooterSubsystem.getTopTalon().set(ControlMode.Velocity, UnitConversions.rpmToRawSensorVelocity(topTargetRPM, 2048));
    shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, UnitConversions.rpmToRawSensorVelocity(bottomTargetRPM, 2048));

    SmartDashboard.putBoolean("Top On Target", shooterSubsystem.topOnTarget());
    SmartDashboard.putBoolean("Bottom on Target", shooterSubsystem.bottomOnTarget());
    SmartDashboard.putBoolean("Both on Target", shooterSubsystem.bothOnTarget());
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
