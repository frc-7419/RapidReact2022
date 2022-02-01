package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunRevolver extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private RevolverSub revolver;
  private double power;
  private boolean reversed;
  double coeff;

  public RunRevolver(RevolverSub revolver, double power, boolean reversed) {
    this.revolver = revolver;
    this.power = power;
    this.reversed = reversed;
  }

  @Override
  public void initialize() {
    if(reversed){coeff = -1;}
    else{coeff = 1;}
  }

  @Override
  public void execute() {
    SmartDashboard.putString("revolver", "running");
    revolver.setPower(power * coeff);
  }

  @Override
  public void end(boolean interrupted) {
      revolver.setPower(0);
      revolver.brakeMode();
  }

  @Override
  public boolean isFinished() {
    return false;
  }


}