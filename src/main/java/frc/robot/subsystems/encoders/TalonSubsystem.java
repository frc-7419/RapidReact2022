// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class TalonSubsystem extends SubsystemBase {
  private TalonFX talon;

  public TalonSubsystem() {
    talon = new TalonFX(0);
    talon.configFactoryDefault();
    talon.setInverted(false);
    talon.setSensorPhase(false);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public double getTalonPosition(){
    return talon.getSelectedSensorPosition();
  }

  public double getTalonVelocity(){
    return talon.getSelectedSensorVelocity();
  }

  public void setTalonPosition(double sensorPosition){
    talon.setSelectedSensorPosition(sensorPosition);
  }

  public void setPower(double power){
    talon.set(ControlMode.PercentOutput, power);
  }

  public double inchesToTicks(double inches, int diameter){
    //(ticks per rotation/diameter of wheels)*inches
    //delete the diameter parameter after finding out what it is
    double ticks = (2048 * inches)/(Math.PI * diameter);
    return ticks;
  }

}
