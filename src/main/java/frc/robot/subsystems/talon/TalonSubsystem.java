package frc.robot.subsystems.talon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonSubsystem extends SubsystemBase {
  private TalonSRX talonSRX;
  private TalonSRX talonFollowerLeft;
  
  public TalonSubsystem() {
    talonFollowerLeft = new TalonSRX(50);
    talonSRX = new TalonSRX(51);

    talonFollowerLeft.follow(talonSRX);
    talonSRX.setInverted(true);
    talonFollowerLeft.setInverted(InvertType.OpposeMaster);

    talonSRX.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
  }

  @Override
  public void periodic() {
  }

  public TalonSRX getTalonSRX() {
    return talonSRX;
  }


  public void setPower(double power){
    talonSRX.set(ControlMode.PercentOutput, power);
  }

}
