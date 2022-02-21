package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class RunIntakeWithDriveBase extends CommandBase{
  private IntakeSubsystem intakeSubsystem;
  private DriveBaseSubsystem driveBaseSubsystem;
  private XboxController joystick;

  private SimpleMotorFeedforward intakeFeedforward;
  
  public RunIntakeWithDriveBase(IntakeSubsystem intakeSubsystem, DriveBaseSubsystem driveBaseSubsystem, XboxController joystick) {
    this.intakeSubsystem = intakeSubsystem;
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.joystick = joystick;
    intakeFeedforward = new SimpleMotorFeedforward(RobotConstants.IntakeKs, RobotConstants.IntakeKv);
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double driveBaseRawVelocity = driveBaseSubsystem.getLeftVelocity();

    if (driveBaseRawVelocity > 0) {
      intakeSubsystem.setPIDFConstants(PIDConstants.IntakeKp, PIDConstants.IntakeKd, PIDConstants.IntakeKi, intakeFeedforward.calculate(driveBaseRawVelocity * 2));
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