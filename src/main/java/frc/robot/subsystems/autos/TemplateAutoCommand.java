package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TemplateAutoCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public TemplateAutoCommand() {
    // uses addRequirements() instead of requires()
    addRequirements();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
