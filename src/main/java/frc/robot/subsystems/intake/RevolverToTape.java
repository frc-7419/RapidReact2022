package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.colorSensor.RevColorDistanceSub;
import frc.robot.subsystems.intake.RevolverSub;

public class RevolverToTape extends CommandBase {
 
  private RevColorDistanceSub colorSensor;
  private RevolverSub revolver;
  private double red;

  public RevolverToTape(RevColorDistanceSub colorSensor, RevolverSub revolver) {
    this.colorSensor = colorSensor;
    this.revolver = revolver;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    revolver.setPower(-.7);
    red = colorSensor.getRgb()[0];
  }

  @Override
  public void end(boolean interrupted) {
    revolver.setPower(0);
    revolver.brakeMode();
  }

  @Override
  public boolean isFinished() {
    return red > .3;
  }
}
