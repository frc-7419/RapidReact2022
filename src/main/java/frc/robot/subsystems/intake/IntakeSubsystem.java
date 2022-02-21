package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class IntakeSubsystem extends SubsystemBase{
    private CANSparkMax intake;
    private RelativeEncoder intakeEncoder;
    private SparkMaxPIDController intakePIDController;
  
    public IntakeSubsystem() {
      // initialize intake spark
      intake = new CANSparkMax(CanIds.intakeSpark.id, MotorType.kBrushless);
  
      intake.restoreFactoryDefaults();
  
      // encoder object created to display position values
      intakeEncoder = intake.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
  
      // set encoder as feedback device for pid controller
      intakePIDController.setFeedbackDevice(intakeEncoder);
  
      // set PID controller
      intakePIDController = intake.getPIDController();
    }
  
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Intake Velocity", intakeEncoder.getVelocity());
    }
  
    public void setPIDFConstants(double kP, double kD, double kI, double kF) {
      intakePIDController.setP(kP);
      intakePIDController.setI(kI);
      intakePIDController.setD(kD);
      intakePIDController.setFF(kF);
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
      return intakeEncoder;
    }
  
    public SparkMaxPIDController getIntakePIDController() {
      return intakePIDController;
    }
}