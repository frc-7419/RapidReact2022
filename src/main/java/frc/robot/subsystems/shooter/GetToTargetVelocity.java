package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class GetToTargetVelocity extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private ShooterSubsystem shooterSubsystem;
  private LimelightSubsystem limelight;
  
  private double kF;

  private double targetRPM = UnitConversions.mPSToRPM(shooterSubsystem.getV0(), RobotConstants.shooterRadius);

  public GetToTargetVelocity(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelight) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelight = limelight;
  }

  @Override
  public void initialize() {

      SmartDashboard.putString("shooter", "ramping up");
      shooterSubsystem.setkF(shooterSubsystem.computekF(targetRPM));
      
      double kP = 0;
      double kI = 0;
      double kD = 0;

      shooterSubsystem.setPIDF(kP, kI, kD, shooterSubsystem.getkF());
      shooterSubsystem.setTargetRawSpeed(targetRPM);
  }

  @Override
  public void execute() {
    shooterSubsystem.getShooterTalon().set(ControlMode.Velocity, targetRPM);
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
