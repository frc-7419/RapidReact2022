package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limitswitch.LimitSwitchSubsystem;
import com.team7419.Initers;


public class MotorRetractIntake extends CommandBase {

  private LimitSwitchSubsystem elevatorLimitSwitchSubsystem;
  private LimitSwitchSubsystem intakeLimitSwitchSubsystem;
  private IntakeSubsystem intakeSubsystem;

  public MotorRetractIntake(LimitSwitchSubsystem elevatorLimitSwitchSubsystem, LimitSwitchSubsystem intakeLimitSwitchSubsystem, IntakeSubsystem intakeSubsystem) {
    this.elevatorLimitSwitchSubsystem = elevatorLimitSwitchSubsystem;
    this.intakeLimitSwitchSubsystem = intakeLimitSwitchSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    Initers.initVictors(intakeSubsystem.getLeft(), intakeSubsystem.getRight());
    addRequirements(elevatorLimitSwitchSubsystem, intakeLimitSwitchSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (elevatorLimitSwitchSubsystem.get()) {
      if (!intakeLimitSwitchSubsystem.get()) {
        intakeSubsystem.setPower(-intakeSubsystem.getPower());
      }
      else {
        intakeSubsystem.setPower(0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
