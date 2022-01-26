package frc.robot.subsystems.talon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limitswitch.LimitswitchSubsystem;


public class RunMotorWithLimitSwitch extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private LimitswitchSubsystem limitSwitchSubsystem;
  private TalonSubsystem talonSubsystem;

  public RunMotorWithLimitSwitch(LimitswitchSubsystem limitSwitchSubsystem, TalonSubsystem talonSubsystem) {
    this.limitSwitchSubsystem = limitSwitchSubsystem;
    this.talonSubsystem = talonSubsystem;
    // uses addRequirements() instead of requires()
    addRequirements(talonSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // if the limit switch is pressed, set power to zero and brake
    if (!limitSwitchSubsystem.get()){
      talonSubsystem.setPower(0);
      talonSubsystem.brake();
    }
    else {
      talonSubsystem.setPower(0.1);
    }
    SmartDashboard.putNumber("velocity", talonSubsystem.getTalon().getSelectedSensorVelocity());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
