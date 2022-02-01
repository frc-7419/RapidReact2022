package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GetToTargetVelocity extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private ShooterSub shooter;
  private double kF;

  private double target;
  private double steadyLoops = 0;
  private boolean stable = true;

  public GetToTargetVelocity(ShooterSub shooter, double target) {
    this.shooter = shooter;
    this.target = target;
  }

  @Override
  public void initialize() {

      SmartDashboard.putString("shooter", "ramping up");
      shooter.setkF(shooter.lookUpkF(target));
      
      double kP = 0;
      double kI = 0;
      double kD = 0;
      // double[] gains = dashboard.getRampingGains();
      shooter.setPIDF(kP, kI, kD, shooter.getkF());
      shooter.setTargetRawSpeed(target);
      // shooter.setControlMethod(ControlMethod.SPIN_UP);
  }

  @Override
  public void execute() {
    shooter.talon.set(ControlMode.Velocity, target);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
