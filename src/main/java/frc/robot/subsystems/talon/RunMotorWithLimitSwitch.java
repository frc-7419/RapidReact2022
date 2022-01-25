package frc.robot.subsystems.limitswitch;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.talon.TalonSubsystem;


public class RunMotorWithLimitSwitch extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private LimitSwitchSubsystem limitSwitchSubsystem;
  private TalonSubsystem talonSubsystem;

  public RunMotorWithLimitSwitch(LimitSwitchSubsystem limitSwitchSubsystem, TalonSubsystem talonSubsystem) {
    this.limitSwitchSubsystem = limitSwitchSubsystem;
    this.talonSubsystem = talonSubsystem;
    // uses addRequirements() instead of requires()
    addRequirements(limitSwitchSubsystem, talonSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (limitSwitchSubsystem.getLimitSwitch().get()){
      talonSubsystem.setPower(0.2);
    }
    // add an else statement that brakes the motor
    else talonSubsystem.setNeutralMode(NeutralMode.Brake);
    }
    // ^ this sets the power to 0 but doesn't BRAKE it. check assignments for help
    
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
