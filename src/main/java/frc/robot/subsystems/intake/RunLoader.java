package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunLoader extends CommandBase{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private LoaderSub loader;
  private double power;
  private boolean reversed;
  private double negativeSign;
  
 /**
  * 
  * @param loader
  * @param power can have dashboard dependencies
  * @param reversed
  */
  public RunLoader(LoaderSub loader, double power, boolean reversed) {
    this.loader = loader;
    this.power = power;
    this.reversed = reversed;
  }

  @Override
  public void initialize() {
    if(reversed){negativeSign = -1;}
    else{negativeSign = 1;}
  }

  @Override
  public void execute() {
    loader.setPower(negativeSign * power);
}

  @Override
  public void end(boolean interrupted) {
      loader.setPower(0);
      loader.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }


}