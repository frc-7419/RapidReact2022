package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntake extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeSub intake;
  private double power;
  private XboxController joystick;
  
  /**
   * 
   * @param intake
   * @param joystick
   * @param power
   */
  public RunIntake(IntakeSub intake, XboxController joystick, double power) {
    this.intake = intake;
    this.power = power;
    this.joystick = joystick;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.setPower(power);

    /* untested: sets intake power to values from joystick
    assuming that left trigger raw vals are negative, but im not actually sure about that */
    // intake.setPower(joystick.getLeftTrig() + joystick.getRightTrig());
}

  @Override
  public void end(boolean interrupted) {
      intake.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }


}