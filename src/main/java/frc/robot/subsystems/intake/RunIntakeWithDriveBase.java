package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class RunIntakeWithDriveBase extends CommandBase{
  private IntakeSubsystem intakeSubsystem;
  private DriveBaseSubsystem driveBaseSubsystem;
  private XboxController joystick;
  private double intakeMultiplier;
  
  public RunIntakeWithDriveBase(IntakeSubsystem intakeSubsystem, DriveBaseSubsystem driveBaseSubsystem, XboxController joystick, double intakeMultiplier) {
    this.intakeSubsystem = intakeSubsystem;
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.joystick = joystick;
    this.intakeMultiplier = intakeMultiplier;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double driveBaseVelocity = driveBaseSubsystem.getLeftVelocity();
    
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