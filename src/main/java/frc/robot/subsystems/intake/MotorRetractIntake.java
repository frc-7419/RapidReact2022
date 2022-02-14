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
  private LimitSwitchSubsystem limitSwitchSubsystem;
  private VictorSPX left;
  private VictorSPX right;

  public MotorRetractIntake(ElevatorSubsystem elevatorSubsystem, LimitSwitchSubsystem limitSwitchSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.limitSwitchSubsystem = limitSwitchSubsystem;
    left = new VictorSPX(CanIds.intakeVictor.id);
    right = new VictorSPX(CanIds.intakeVictor.id);
    Initers.initVictors(left, right);
    addRequirements(elevatorSubsystem, limitSwitchSubsystem);
  }

    public VictorSPX getLeft() {
        return left;
    }

    public VictorSPX getRight() {
      return right;
    }

    public void setPower(double power) {
        left.set(ControlMode.PercentOutput, power);
        right.set(ControlMode.PercentOutput, power);
    }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (limitSwitchSubsystem.get()) {

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
