package frc.robot;

import com.team7419.joystick.DoubleButton;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.SetLEDColor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PowerConstants;
import frc.robot.commands.RunIntakeAndLoaderWithJoystick;
import frc.robot.subsystems.arms.ArmsSubsystem;
import frc.robot.subsystems.arms.CoastArms;
import frc.robot.subsystems.arms.RunArmsWithJoystick;
import frc.robot.subsystems.autos.OneBallAuto;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.NewArcadeDrive;
import frc.robot.subsystems.drive.NewDriveBaseSubsystem;
import frc.robot.subsystems.drive.NewTankDrive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.MaintainElevatorPosition;
import frc.robot.subsystems.elevator.RunElevatorWithJoystick;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeederWithJoystick;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.DeployIntakeWithJoystick;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.RunShooterWithJoystick;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.RunTurretWithJoystick;
import frc.robot.subsystems.turret.TurretSubsystem;


public class RobotContainer {
  private final XboxController joystick1 = new XboxController(0);
  private final XboxController joystick2 = new XboxController(1);
  private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmsSubsystem armsSubsystem = new ArmsSubsystem();

  private final DeployIntakeWithJoystick deployIntakeWithJoystick = new DeployIntakeWithJoystick(intakeSolenoidSubsystem, joystick2);
  private final RunTurretWithJoystick runTurretWithJoystick = new RunTurretWithJoystick(turretSubsystem, joystick2, 0.2);
  private final RunIntakeAndLoaderWithJoystick runIntakeAndLoaderWithJoystick = new RunIntakeAndLoaderWithJoystick(joystick1, intakeSubsystem, loaderSubsystem, 1);
  private final AlignTurretDefault alignTurretDefault = new AlignTurretDefault(turretSubsystem, limelightSubsystem);
  // private final RunShooterWithJoystick runShooterWithJoystick = new RunShooterWithJoystick(shooterSubsystem, joystick2);
  private final RunFeederWithJoystick runFeederWithJoystick = new RunFeederWithJoystick(feederSubsystem, joystick1, 1);
  private final RunElevatorWithJoystick runElevatorWithJoystick = new RunElevatorWithJoystick(elevatorSubsystem, joystick2);
  private final RunArmsWithJoystick runArmsWithJoystick = new RunArmsWithJoystick(armsSubsystem, joystick2);
  // private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick1, driveBaseSubsystem, 
  // PowerConstants.DriveBaseStraight, PowerConstants.DriveBaseTurn);

  private final NewDriveBaseSubsystem newDriveBaseSubsystem = new NewDriveBaseSubsystem();
  // private final NewTankDrive newTankDrive = new NewTankDrive(joystick1, newDriveBaseSubsystem, .75);
  private final NewArcadeDrive newArcadeDrive = new NewArcadeDrive(joystick1, newDriveBaseSubsystem, 0.95, 0.75);

  // auto
  private SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final OneBallAuto oneBallAuto = new OneBallAuto(driveBaseSubsystem, gyroSubsystem, shooterSubsystem, limelightSubsystem, feederSubsystem, loaderSubsystem);

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    // align turret
    new JoystickButton(joystick2, XboxController.Button.kLeftBumper.value)
    .whileHeld(new AlignTurretDefault(turretSubsystem, limelightSubsystem));

    // edge of tarmac 
      // new JoystickButton(joystick2, XboxController.Button.kX.value)
      // .whileHeld(new GetToTargetVelocity(shooterSubsystem, 9850, 6150, 0.0485, 0.0495));

      new JoystickButton(joystick2, XboxController.Button.kX.value)
      .whileHeld(new GetToTargetVelocity(shooterSubsystem, 8530, 8160, 0.04876, 0.04772));
    
    // toggle to maintain elevator position
    new DoubleButton(
      new JoystickButton(joystick2, XboxController.Button.kA.value),
      new JoystickButton(joystick2, XboxController.Button.kB.value))
      .toggleWhenPressed(new MaintainElevatorPosition(elevatorSubsystem));

      // lower hub shot, consistent
    new JoystickButton(joystick2, XboxController.Button.kA.value)
    .whileHeld(new GetToTargetVelocity(shooterSubsystem, 3500, 5450, 0.042, 0.0475));
      
    // long
    // new JoystickButton(joystick2, XboxController.Button.kY.value)
    // .whileHeld(new GetToTargetVelocity(shooterSubsystem, 11500, 6500, 0.04775, 0.0477425));

    // new long
    new JoystickButton(joystick2, XboxController.Button.kY.value)
    .whileHeld(new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049));


    new JoystickButton(joystick2, XboxController.Button.kRightBumper.value)
    .whileHeld(new CoastArms(armsSubsystem));
    
  }

  private void smartDashboardBindings() {}

  private void configureAutoSelector() {
    autonChooser.setDefaultOption("Preload Default", oneBallAuto);
    // autonChooser.addOption("2 Ball", twoBallAuto);
    // autonChooser.addOpton("3 Ball", threeBallAuto);
    // autonChooser.addOption("5 Ball", fiveBallAuto);
    SmartDashboard.putData(autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
    
  public void setDefaultCommands() {
    // driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    newDriveBaseSubsystem.setDefaultCommand(newArcadeDrive);
    intakeSolenoidSubsystem.setDefaultCommand(deployIntakeWithJoystick);
    intakeSubsystem.setDefaultCommand(runIntakeAndLoaderWithJoystick);
    loaderSubsystem.setDefaultCommand(runIntakeAndLoaderWithJoystick); 
    feederSubsystem.setDefaultCommand(runFeederWithJoystick);
    // shooterSubsystem.setDefaultCommand(runShooterWithJoystick);
    turretSubsystem.setDefaultCommand(runTurretWithJoystick);
    // elevatorSubsystem.setDefaultCommand(runElevatorWithJoystick);
    armsSubsystem.setDefaultCommand(runArmsWithJoystick);
  }
}

