package frc.robot.subsystems.revMagneticLimitSwitch;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.talon.TalonSubsystem;


public class RunMotorWithLimitSwitch extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private RevMagneticLimitSwitchSubsystem revMagneticLimitSwitchSubsystem;
  private TalonSubsystem talonSubsystem;

  public RunMotorWithLimitSwitch(RevMagneticLimitSwitchSubsystem revMagneticLimitSwitchSubsystem, TalonSubsystem talonSubsystem) {
    this.revMagneticLimitSwitchSubsystem = revMagneticLimitSwitchSubsystem;
    this.talonSubsystem = talonSubsystem;
    // uses addRequirements() instead of requires()
    addRequirements(revMagneticLimitSwitchSubsystem, talonSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      if (revMagneticLimitSwitchSubsystem.getRevMagneticLimitSwitch().get()){
        talonSubsystem.setPower(0.2);
      }
      // add an else statement that brakes the motor
      else {
        talonSubsystem.brake();
      }
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
