package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax.ControlType;
import com.team7419.math.UnitConversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class RunIntakeWithDriveBase extends CommandBase{
  private IntakeSubsystem intakeSubsystem;
  private DriveBaseSubsystem driveBaseSubsystem;

  private SimpleMotorFeedforward intakeFeedforward;
  
  public RunIntakeWithDriveBase(IntakeSubsystem intakeSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.driveBaseSubsystem = driveBaseSubsystem;
    intakeFeedforward = new SimpleMotorFeedforward(RobotConstants.IntakeKs, RobotConstants.IntakeKv);
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double driveBaseLinearVelocity = UnitConversions.rpmToMPS(UnitConversions.rawSensorVelocityToRPM(driveBaseSubsystem.getLeftVelocity(), 4096), RobotConstants.intakeWheelRadius);

    if (driveBaseLinearVelocity > 0) {
      intakeSubsystem.setPIDFConstants(PIDConstants.IntakeKp, PIDConstants.IntakeKd, PIDConstants.IntakeKi, intakeFeedforward.calculate(driveBaseLinearVelocity));
      intakeSubsystem.getIntakePIDController().setReference(UnitConversions.mpsToRawSensorVelocity(2*driveBaseLinearVelocity, 4096, RobotConstants.intakeWheelRadius), ControlType.kVelocity);
    }
    else {
      intakeSubsystem.setPower(0.9);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }


}