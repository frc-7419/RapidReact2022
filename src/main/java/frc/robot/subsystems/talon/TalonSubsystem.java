package frc.robot.subsystems.talon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonSubsystem extends SubsystemBase {
  private TalonSRX talonSRX;
  
  public TalonSubsystem() {
    talonSRX = new TalonSRX(51);
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
