package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class GetToTargetVelocity extends CommandBase {

  private ShooterSubsystem shooterSubsystem;
  private LimelightSubsystem limelightSubsystem;

  private double kP;
  private double kI;
  private double kD;
  private double kF;

  private double initialVelocity;
  private double targetRPM;
  // private double topTargetRPM;
  // private double bottomTargetRPM;
  
  public GetToTargetVelocity(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("shooter", "ramping up");

    initialVelocity = Math.sqrt(LimelightConstants.g/(2*limelightSubsystem.getA()*(Math.pow(Math.cos(Math.toRadians(limelightSubsystem.getBeta())),2))));
    
    targetRPM = UnitConversions.mPSToRPM(initialVelocity, RobotConstants.topShooterWheelRadius);
    
    shooterSubsystem.setkF(shooterSubsystem.computekF(targetRPM));

    shooterSubsystem.setPIDF(kP, kI, kD, shooterSubsystem.getkF());
    shooterSubsystem.setTargetRawVelocity(targetRPM);
  }

  @Override
  public void execute() {
    shooterSubsystem.getTopTalon().set(ControlMode.Velocity, targetRPM);
    shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, targetRPM);
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
