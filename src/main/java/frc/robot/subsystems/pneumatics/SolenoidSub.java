// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SolenoidSub extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  DoubleSolenoid solenoid;
  //boolean reversed;
  

  public SolenoidSub() {
    solenoid = new DoubleSolenoid(1,0,1);
    //this.reversed = reversed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void actuateSolenoid() {
    solenoid.set(Value.kForward);
  }

  public void retractSolenoid() {
    solenoid.set(Value.kReverse);
  }

  public void stopSolenoid() {
    solenoid.set(Value.kOff);
  }
}
