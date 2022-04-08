package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arms.ArmsSubsystem;
import frc.robot.subsystems.autos.SvrThreeBall;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.TurretSubsystem;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final XboxController joystick = new XboxController(0);
  private final XboxController joystick1 = new XboxController(0);
  private final XboxController joystick2 = new XboxController(1);
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmsSubsystem armsSubsystem = new ArmsSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final SvrThreeBall svrThreeBallNew = new SvrThreeBall(turretSubsystem, limelightSubsystem, shooterSubsystem, loaderSubsystem, feederSubsystem, driveBaseSubsystem, gyroSubsystem, intakeSubsystem, intakeSolenoidSubsystem, 
  ledSubsystem);
  

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    // align turret
    new JoystickButton(joystick2, XboxController.Button.kLeftBumper.value)
    .whileHeld(new AlignTurretDefault(turretSubsystem, limelightSubsystem));
    
  }

  private void smartDashboardBindings() {
    // SmartDashboard.putNumber("tTargetVelocity", 10);
    // SmartDashboard.putNumber("bTargetVelocity", 10);

    // SmartDashboard.putNumber("bKp", PIDConstants.BottomShooterkP);
    // SmartDashboard.putNumber("bKi", PIDConstants.BottomShooterkI);

    // SmartDashboard.putNumber("tKp", PIDConstants.TopShooterkP);
    // SmartDashboard.putNumber("tKi", PIDConstants.TopShooterkI);
  }

  private void configureAutoSelector() {
    // autonChooser.setDefaultOption("Preload Default", oneBallAuto);
    // // autonChooser.addOption("2 Ball", twoBallAuto);
    // // autonChooser.addOpton("3 Ball", threeBallAuto);
    // // autonChooser.addOption("5 Ball", fiveBallAuto);
    // SmartDashboard.putData(autonChooser);
  }

  public Command getAutonomousCommand() {
    //return shootGetSecondBallShoot;
    return svrThreeBallNew;
    //return coast;
    //return turn180;
  }

  // set default commands here
  public void setDefaultCommands() {}
}

