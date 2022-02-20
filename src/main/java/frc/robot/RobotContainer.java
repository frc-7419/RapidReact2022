package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.Constants.PowerConstants;
import frc.robot.subsystems.intake.DeployIntake;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.pneumatics.CompressorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();
  // private final CompressorSubsystem compressorSubsystem = new CompressorSubsystem();

  // private final DeployIntake deployIntake = new DeployIntake(intakeSolenoidSubsystem, joystick);
  private final RunIntake runIntake = new RunIntake(intakeSubsystem, joystick, PowerConstants.intakeMultiplier);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

  public void setDefaultCommands(){
    intakeSubsystem.setDefaultCommand(runIntake);
    // intakeSolenoidSubsystem.setDefaultCommand(deployIntake);
  }

  
}
