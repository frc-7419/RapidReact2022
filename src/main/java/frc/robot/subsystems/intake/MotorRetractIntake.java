package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.limitswitch.LimitSwitchSubsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team7419.Initers;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.subsystems.pneumatics.SolenoidSubsystem;


public class MotorRetractIntake extends CommandBase {

  private ElevatorSubsystem elevatorSubsystem;
  private LimitSwitchSubsystem elevatorLimitSwitchSubsystem;
  private LimitSwitchSubsystem intakeLimitSwitchSubsystem;
  private VictorSPX left;
  private VictorSPX right;
  private IntakeSubsystem intakeSubsystem;

  public MotorRetractIntake(ElevatorSubsystem elevatorSubsystem, LimitSwitchSubsystem elevatorLimitSwitchSubsystem, LimitSwitchSubsystem intakeLimitSwitchSubsystem, IntakeSubsystem intakeSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorLimitSwitchSubsystem = elevatorLimitSwitchSubsystem;
    this.elevatorLimitSwitchSubsystem = intakeLimitSwitchSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    left = new VictorSPX(CanIds.intakeVictor.id);
    right = new VictorSPX(CanIds.intakeVictor.id);
    Initers.initVictors(left, right);
    addRequirements(elevatorSubsystem, elevatorLimitSwitchSubsystem, intakeLimitSwitchSubsystem, intakeSubsystem);
  }

  public VictorSPX getLeft() {
    return left;
  }

  public VictorSPX getRight() {
    return right;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (elevatorLimitSwitchSubsystem.get() && !intakeLimitSwitchSubsystem.get()) {
      intakeSubsystem.setPower(-intakeSubsystem.getPower());
    }
    else {
      intakeSubsystem.setPower(0);
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
