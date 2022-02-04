package frc.robot.subsystems.intakeCommandColorSensor.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PowerConstants;

public class IntakeDefault extends CommandBase{
  private IntakeSubsystem intakeSubsystem;
  private XboxController joystick;
  
  public IntakeDefault(IntakeSubsystem intakeSubsystem, XboxController joystick) {
    this.intakeSubsystem = intakeSubsystem;
    this.joystick = joystick;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // assumption being that left trigger vals are negative
    if (Math.abs(joystick.getLeftTriggerAxis()) > 0) {
      intakeSubsystem.setPower(PowerConstants.intakeMultiplier * joystick.getLeftTriggerAxis());
    } else if (Math.abs(joystick.getRightTriggerAxis()) > 0) {
      intakeSubsystem.setPower(PowerConstants.intakeMultiplier * joystick.getRightTriggerAxis());
    } else {
      intakeSubsystem.setPower(0);
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