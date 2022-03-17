// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.turret;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxLimitSwitch;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CanIds;

// public class TurretSubsystem extends SubsystemBase {
//   private CANSparkMax turret;
//   private SparkMaxLimitSwitch forwardLimitSwitch;
//   private SparkMaxLimitSwitch reverseLimitSwitch;

//   private RelativeEncoder encoder;
//   private boolean forwardLimitDetected = false;
//   private boolean reverseLimitDetected = false;

//   public TurretSubsystem() {
//     turret = new CANSparkMax(CanIds.turretSpark.id, MotorType.kBrushless);
//     forwardLimitSwitch = turret.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//     reverseLimitSwitch = turret.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//     encoder = turret.getEncoder();

//     turret.restoreFactoryDefaults();
    
//     forwardLimitSwitch.enableLimitSwitch(false);
//     reverseLimitSwitch.enableLimitSwitch(false);

//     // turret.burnFlash();

//     SmartDashboard.putBoolean("Forward Limit Enabled", forwardLimitSwitch.isLimitSwitchEnabled());
//     SmartDashboard.putBoolean("Reverse Limit Enabled", reverseLimitSwitch.isLimitSwitchEnabled());
//   }

//   @Override
//   public void periodic() {
//     if (getReverseLimitSwitch().isPressed() && !reverseLimitDetected) {
//       reverseLimitDetected = true;
//       turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
//       turret.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)encoder.getPosition());
//     } 
//     if (getForwardLimitSwitch().isPressed() && !forwardLimitDetected) {
//       forwardLimitDetected = true;
//       turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
//       turret.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)encoder.getPosition());

//     } 
//     SmartDashboard.putBoolean("Reverse Limit Detected", reverseLimitDetected);

//     SmartDashboard.putBoolean("Forward Limit Detected", forwardLimitDetected);

//     SmartDashboard.putBoolean("Forward Limit Switch", forwardLimitSwitch.isPressed());
//     SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimitSwitch.isPressed());
//     // SmartDashboard.putNumber("Turret Encoder Position", encoder.getPosition());
//   }

//   public void setPower(double power) {
//     coast();
//     turret.set(power);
//   }

//   public void brake() {
//     turret.setIdleMode(IdleMode.kBrake);
//   }
//   public void coast() {
//     turret.setIdleMode(IdleMode.kCoast);
//   }

//   public SparkMaxLimitSwitch getForwardLimitSwitch() {
//     return forwardLimitSwitch;
//   } 
//   public SparkMaxLimitSwitch getReverseLimitSwitch() {
//     return reverseLimitSwitch;
//   }
// }
