package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonSRX elevatorRight;
  private TalonSRX elevatorFollowerLeft;
  
  public ElevatorSubsystem() {
    elevatorFollowerLeft = new TalonSRX(50);
    elevatorRight = new TalonSRX(51);

    elevatorFollowerLeft.follow(elevatorRight);
    elevatorRight.setInverted(true);
    elevatorFollowerLeft.setInverted(InvertType.OpposeMaster);

    elevatorRight.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
  }

  @Override
  public void periodic() {
  }

  public TalonSRX getTalonSRX() {
    return elevatorRight;
  }


  public void setPower(double power){
    elevatorRight.set(ControlMode.PercentOutput, power);
  }

}
