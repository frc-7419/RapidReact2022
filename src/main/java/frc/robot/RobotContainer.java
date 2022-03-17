package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.shooter.RunShooterWithJoystick;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.RunTurretWithJoystick;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.Constants.PowerConstants;
import frc.robot.commands.RunIntakeAndLoaderWithJoystick;
import frc.robot.subsystems.arms.ArmsSubsystem;
import frc.robot.subsystems.arms.RunArmsWithJoystick;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.RunElevatorWithJoystick;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeederWithJoystick;
import frc.robot.subsystems.intake.DeployIntakeWithJoystick;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);
  private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmsSubsystem armsSubsystem = new ArmsSubsystem();

  private final DeployIntakeWithJoystick deployIntakeWithJoystick = new DeployIntakeWithJoystick(intakeSolenoidSubsystem, joystick);
  private final RunTurretWithJoystick runTurretWithJoystick = new RunTurretWithJoystick(turretSubsystem, joystick, 0.1);
  private final RunIntakeAndLoaderWithJoystick runIntakeAndLoaderWithJoystick = new RunIntakeAndLoaderWithJoystick(joystick, intakeSubsystem, loaderSubsystem, 1);
  private final AlignTurretDefault alignTurretDefault = new AlignTurretDefault(turretSubsystem, limelightSubsystem);
  private final RunShooterWithJoystick runShooterWithJoystick = new RunShooterWithJoystick(shooterSubsystem, joystick);
  private final RunFeederWithJoystick runFeederWithJoystick = new RunFeederWithJoystick(feederSubsystem, joystick, 0.15);
  private final RunElevatorWithJoystick runElevatorWithJoystick = new RunElevatorWithJoystick(elevatorSubsystem, joystick);
  private final RunArmsWithJoystick runArmsWithJoystick = new RunArmsWithJoystick(armsSubsystem, joystick);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick, driveBaseSubsystem, 
  PowerConstants.DriveBaseStraight, PowerConstants.DriveBaseTurn);

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {}

  private void smartDashboardBindings() {}

  public Command getAutonomousCommand() {
    return alignTurretDefault;
  }
    
  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    intakeSolenoidSubsystem.setDefaultCommand(deployIntakeWithJoystick);
    intakeSubsystem.setDefaultCommand(runIntakeAndLoaderWithJoystick);
    loaderSubsystem.setDefaultCommand(runIntakeAndLoaderWithJoystick); 
    shooterSubsystem.setDefaultCommand(runShooterWithJoystick);
    // turretSubsystem.setDefaultCommand(alignTurretDefault);
    turretSubsystem.setDefaultCommand(runTurretWithJoystick);
    feederSubsystem.setDefaultCommand(runFeederWithJoystick);

    elevatorSubsystem.setDefaultCommand(runElevatorWithJoystick);
    armsSubsystem.setDefaultCommand(runArmsWithJoystick);
  }


}
