// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {
  private TalonSRX elevatorLeft;
  private TalonSRX elevatorRight;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorLeft = new TalonSRX(50);
    elevatorRight = new TalonSRX(51);
    elevatorRight.setInverted(true);
    elevatorRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public TalonSRX getElevatorLeft(){
    return elevatorLeft;
  }

  public TalonSRX getElevatorRight(){
    return elevatorRight;
  }

  public void setPower(double power) {
    elevatorLeft.set(ControlMode.PercentOutput, power);
    elevatorRight.set(ControlMode.PercentOutput, power);
  }


  public void brake() {
    elevatorLeft.setNeutralMode(NeutralMode.Brake);
    elevatorRight.setNeutralMode(NeutralMode.Brake);
  }

  public double inchesToTicks(double inches, double diameter, double gearRatioMultiplier){
    //(ticks per rotation/diameter of wheels)*inches
    //delete the diameter parameter after finding out what it is
    double ticks = (1024 * inches)/(Math.PI * diameter * gearRatioMultiplier);
    
    return ticks;
  }
}