// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SolenoidSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private Solenoid solenoid;
  //boolean reversed;
  

  public SolenoidSubsystem(int id, int channel) {
    solenoid = new Solenoid(id, PneumaticsModuleType.CTREPCM, channel); //id=1, channel=0
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
    solenoid.set(true);
  }

  public void retractSolenoid() {
    solenoid.set(false);
  }
  public void toggleSolenoid() {
    solenoid.toggle();
  }

//  public void stopSolenoid() {
//    solenoid.set(Value.kOff);
//  }
}
