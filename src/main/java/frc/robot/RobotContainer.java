package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.intake.RunIntakeWithDriveBase;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.Constants.PowerConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.intake.DeployIntake;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.pneumatics.CompressorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();

  private final DeployIntake deployIntake = new DeployIntake(intakeSolenoidSubsystem, joystick);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick, XboxController.Button.kX.value).toggleWhenPressed(new RunIntakeWithDriveBase(intakeSubsystem, driveBaseSubsystem));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

  public void setDefaultCommands(){
    intakeSolenoidSubsystem.setDefaultCommand(deployIntake);
  }

  
}
