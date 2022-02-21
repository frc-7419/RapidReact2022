package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoaderWithJoystick;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.Constants.PowerConstants;
import frc.robot.snippets.RunIntakeAndLoaderWithJoystick;
import frc.robot.subsystems.intake.DeployIntake;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.pneumatics.CompressorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();

  private final DeployIntake deployIntake = new DeployIntake(intakeSolenoidSubsystem, joystick);
  private final RunIntakeWithJoystick runIntake = new RunIntakeWithJoystick(intakeSubsystem, joystick, PowerConstants.intakeMultiplier);
  private final RunLoaderWithJoystick runLoader = new RunLoaderWithJoystick(loaderSubsystem, joystick, 1);

  private final RunIntakeAndLoaderWithJoystick runIntakeAndLoaderWithJoystick = new RunIntakeAndLoaderWithJoystick(joystick, intakeSubsystem, loaderSubsystem, 1);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

  public void setDefaultCommands(){
    // intakeSubsystem.setDefaultCommand(runIntake);
    // loaderSubsystem.setDefaultCommand(runLoader);

    intakeSolenoidSubsystem.setDefaultCommand(deployIntake);
    intakeSubsystem.setDefaultCommand(runIntakeAndLoaderWithJoystick);
    loaderSubsystem.setDefaultCommand(runIntakeAndLoaderWithJoystick);
  }

  
}
