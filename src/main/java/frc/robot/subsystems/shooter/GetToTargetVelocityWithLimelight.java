package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class GetToTargetVelocityWithLimelight extends CommandBase {

  private ShooterSubsystem shooterSubsystem;
  private LimelightSubsystem limelightSubsystem;

  private double kP;
  private double kI;
  private double tkF;
  private double bkF;

  private double initialVelocity;

  // private double targetRPM;
  private double topTargetRPM;
  private double bottomTargetRPM;

  
  public GetToTargetVelocityWithLimelight(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    // SmartDashboard.putBoolean("Shooter Running", false);

    // redo math?
    
    topTargetRPM = UnitConversions.mpsToRPM(initialVelocity, RobotConstants.topShooterWheelRadius);
    bottomTargetRPM = UnitConversions.mpsToRPM(initialVelocity, RobotConstants.bottomShooterWheelRadius);

    shooterSubsystem.setTopPIDF(kP, kI, 0, shooterSubsystem.computeTopkF(topTargetRPM));
    shooterSubsystem.setBottomPIDF(kP, kI, 0, shooterSubsystem.computeBottomkF(bottomTargetRPM));
    
    shooterSubsystem.setTopTargetRawVelocity(UnitConversions.rpmToRawSensorVelocity(topTargetRPM, 2048));
    shooterSubsystem.setBottomTargetRawVelocity(UnitConversions.rpmToRawSensorVelocity(bottomTargetRPM, 2048));
    
  }

  @Override
  public void execute() {
    // SmartDashboard.putBoolean("Shooter Running", true);

    shooterSubsystem.getTopTalon().set(ControlMode.Velocity, UnitConversions.rpmToRawSensorVelocity(topTargetRPM, 2048));
    shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, UnitConversions.rpmToRawSensorVelocity(bottomTargetRPM, 2048));

    // SmartDashboard.putBoolean("Top On Target", shooterSubsystem.topOnTarget());
    // SmartDashboard.putBoolean("Bottom on Target", shooterSubsystem.bottomOnTarget());
    // SmartDashboard.putBoolean("Both on Target", shooterSubsystem.bothOnTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.off();
    // SmartDashboard.putBoolean("Shooter Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
