package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.turret.RunTurretWithJoystick;
import frc.robot.subsystems.turret.AlignTurret;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.snippets.DiscardWrongColor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.RunShooterWithJoystick;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.PowerConstants;
import frc.robot.subsystems.intake.DeployIntake;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.colorSensor.RevColorDistanceSub;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.loader.RunLoaderWithJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final RevColorDistanceSub revColorDistanceSub = new RevColorDistanceSub();

  private final DeployIntake deployIntake = new DeployIntake(intakeSolenoidSubsystem, joystick);
  private final RunTurretWithJoystick runTurretWithJoystick = new RunTurretWithJoystick(turretSubsystem, joystick, 0.3);
  private final RunIntake runIntake = new RunIntake(intakeSubsystem, joystick, PowerConstants.intakeMultiplier);
  private final RunLoaderWithJoystick runLoader = new RunLoaderWithJoystick(loaderSubsystem, joystick, 1);
  private final AlignTurret alignTurret = new AlignTurret(turretSubsystem, limelightSubsystem);
  private final RunShooterWithJoystick runShooterWithJoystick = new RunShooterWithJoystick(shooterSubsystem, joystick);
  private final DiscardWrongColor discardWrongColor = new DiscardWrongColor(turretSubsystem, shooterSubsystem, revColorDistanceSub);
  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {}

  private void smartDashboardBindings() {
    SmartDashboard.putNumber("kP", PIDConstants.TurretKp);
    SmartDashboard.putNumber("kI", PIDConstants.TurretKi);
    SmartDashboard.putNumber("kD", PIDConstants.TurretKd);
  }

  public Command getAutonomousCommand() {
    return alignTurret;
  }
    
  public void setDefaultCommands(){
    intakeSubsystem.setDefaultCommand(runIntake);
    intakeSolenoidSubsystem.setDefaultCommand(deployIntake);
    loaderSubsystem.setDefaultCommand(runLoader);
    shooterSubsystem.setDefaultCommand(runShooterWithJoystick);
    turretSubsystem.setDefaultCommand(runTurretWithJoystick);
    revColorDistanceSub.setDefaultCommand(discardWrongColor);
  }

}
