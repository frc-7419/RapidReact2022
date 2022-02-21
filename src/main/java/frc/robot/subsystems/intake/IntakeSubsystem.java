// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intake;

  private RelativeEncoder alternateEncoder;
  private SparkMaxPIDController pidController;


  public IntakeSubsystem() {
    // initialize intake spark
    intake = new CANSparkMax(CanIds.intakeSpark.id, MotorType.kBrushless); // insert new CAN id for intake neo

    intake.restoreFactoryDefaults();

    // Encoder object created to display position values, 8192 counts per rev
    alternateEncoder = intake.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

    // set PID controller
    pidController = intake.getPIDController();

    // set encoder as feedback device for pid controller
    pidController.setFeedbackDevice(alternateEncoder);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Position", alternateEncoder.getPosition());
  }

  public void setPIDFConstants(double kP, double kD, double kI, double kF) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setFF(kF);
  }

  public void setPower(double power) {
    intake.set(power);
  }

  public void brake() {
    intake.setIdleMode(IdleMode.kBrake);
  }

  public void coast() {
    intake.setIdleMode(IdleMode.kCoast);
  }

  public CANSparkMax getIntakeMotor() {
    return intake;
  }

  public RelativeEncoder getIntakeEncoder() {
    return alternateEncoder;
  }

  public SparkMaxPIDController getIntakePIDController() {
    return pidController;
  }
}
