package frc.robot;

import com.team7419.joystick.DoubleButton;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import frc.robot.commands.RunIntakeAndLoaderWithJoystick;
import frc.robot.subsystems.arms.ArmsSubsystem;
import frc.robot.subsystems.arms.CoastArms;
import frc.robot.subsystems.arms.RunArmsWithJoystick;
import frc.robot.subsystems.autos.CCCTwoBall;
import frc.robot.subsystems.autos.CCCTwoBallCopy;
import frc.robot.subsystems.autos.OneBallAuto;
import frc.robot.subsystems.autos.OneBallAutoWait;
import frc.robot.subsystems.autos.ThreeBallAutoExactVelocities;
import frc.robot.subsystems.autos.ThreeBallAutoInterpolation;
import frc.robot.subsystems.autos.TwoBallAutoExactVelocities;
import frc.robot.subsystems.autos.TwoBallAutoInterpolation;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.MaintainElevatorPosition;
import frc.robot.subsystems.elevator.RunElevatorWithJoystick;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeederWithJoystick;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.DeployIntakeWithJoystick;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.SetLEDColorWithJoystick;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.SmartLoad;
import frc.robot.subsystems.shooter.GetToTargetVelocityArbitraryFeedforward;
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.BrakeTurret;
import frc.robot.subsystems.turret.RunTurretWithJoystick;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotContainer {
  private final XboxController joystick1 = new XboxController(0);
  private final XboxController joystick2 = new XboxController(1);
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmsSubsystem armsSubsystem = new ArmsSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem();

  private final RunIntakeAndLoaderWithJoystick runIntakeAndLoaderWithJoystick = new RunIntakeAndLoaderWithJoystick(joystick2, intakeSubsystem, loaderSubsystem);
  private final DeployIntakeWithJoystick deployIntakeWithJoystick = new DeployIntakeWithJoystick(intakeSolenoidSubsystem, joystick2);
  private final RunTurretWithJoystick runTurretWithJoystick = new RunTurretWithJoystick(turretSubsystem, limelightSubsystem, joystick1, 0.16);
  private final RunFeederWithJoystick runFeederWithJoystick = new RunFeederWithJoystick(feederSubsystem, joystick1);
  private final RunElevatorWithJoystick runElevatorWithJoystick = new RunElevatorWithJoystick(elevatorSubsystem, joystick2);
  private final RunArmsWithJoystick runArmsWithJoystick = new RunArmsWithJoystick(armsSubsystem, joystick2);
  private final SetLEDColorWithJoystick setLEDColorWithJoystick = new SetLEDColorWithJoystick(ledSubsystem, limelightSubsystem, driveBaseSubsystem, joystick1, joystick2);
  // private final SetLED1Color setled1Color = new SetLED1Color(ledSubsystem, driveBaseSubsystem);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick1, driveBaseSubsystem, 0.6, 0.6);
  // private final SmartShoot smartShoot = new SmartShoot(shooterSubsystem, feederSubsystem, loaderSubsystem, limelightSubsystem, beamBreakSubsystem);
  private final SmartLoad smartLoad = new SmartLoad(feederSubsystem, loaderSubsystem, beamBreakSubsystem, ledSubsystem);
  // auto
  // private SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final OneBallAuto oneBallAuto = new OneBallAuto(driveBaseSubsystem, gyroSubsystem, shooterSubsystem, limelightSubsystem, feederSubsystem, loaderSubsystem, ledSubsystem, turretSubsystem);
  private final OneBallAutoWait oneBallAutoWait = new OneBallAutoWait(driveBaseSubsystem, gyroSubsystem, shooterSubsystem, limelightSubsystem, feederSubsystem, loaderSubsystem, ledSubsystem, turretSubsystem);
  private final TwoBallAutoExactVelocities twoBallAutoExactVelocities = new TwoBallAutoExactVelocities(driveBaseSubsystem, gyroSubsystem, shooterSubsystem, feederSubsystem, loaderSubsystem, intakeSubsystem, turretSubsystem, limelightSubsystem, ledSubsystem, intakeSolenoidSubsystem);
  private final TwoBallAutoInterpolation twoBallAutoInterpolation = new TwoBallAutoInterpolation(driveBaseSubsystem, gyroSubsystem, shooterSubsystem, feederSubsystem, loaderSubsystem, intakeSubsystem, turretSubsystem, limelightSubsystem, ledSubsystem, intakeSolenoidSubsystem);
  private final ThreeBallAutoExactVelocities threeBallAutoExactVelocities = new ThreeBallAutoExactVelocities(turretSubsystem, limelightSubsystem, shooterSubsystem, loaderSubsystem, feederSubsystem, driveBaseSubsystem, gyroSubsystem, intakeSubsystem, intakeSolenoidSubsystem, ledSubsystem);
  private final ThreeBallAutoInterpolation threeBallAutoInterpolation = new ThreeBallAutoInterpolation(turretSubsystem, limelightSubsystem, shooterSubsystem, loaderSubsystem, feederSubsystem, driveBaseSubsystem, gyroSubsystem, intakeSubsystem, intakeSolenoidSubsystem, ledSubsystem);
  private final CCCTwoBall cccTwoBall = new CCCTwoBall(driveBaseSubsystem, gyroSubsystem, shooterSubsystem, limelightSubsystem, feederSubsystem, loaderSubsystem, ledSubsystem, turretSubsystem, intakeSolenoidSubsystem, intakeSubsystem);
  private final CCCTwoBallCopy cccTwoBallCopy = new CCCTwoBallCopy(driveBaseSubsystem, gyroSubsystem, shooterSubsystem, limelightSubsystem, feederSubsystem, loaderSubsystem, ledSubsystem, turretSubsystem, intakeSolenoidSubsystem, intakeSubsystem);

  public RobotContainer() {
    configureButtonBindings();
    // smartDashboardBindings();
    new Thread(() -> {
      UsbCamera cam1 = CameraServer.startAutomaticCapture();
      UsbCamera cam2 = CameraServer.startAutomaticCapture();
      CvSink video1 = CameraServer.getVideo(cam1);
      CvSink video2 = CameraServer.getVideo(cam2);
      cam1.setResolution(400, 320);
      cam2.setResolution(400, 320);
      CvSource combined = CameraServer.putVideo("Front view", 800, 320); //edit these values later

      Mat s1 = new Mat();
      Mat s2 = new Mat();
      Mat o = new Mat();
      byte[] rgb1;
      byte[] rgb2;

      while (!Thread.interrupted()) {
        if (video1.grabFrame(s1)==0 || video2.grabFrame(s2)==0) {
          continue;
        }
        rgb1 = new byte[3];
        rgb2 = new byte[3];
        for (int row=0;row<320;row++) {
          for (int col=0;col<400;col++) {
            s1.get(row, col, rgb1);
            s2.get(row, col, rgb2);
            o.put(row, col, rgb1);
            o.put(row, col+400, rgb2);
          }
        }
        combined.putFrame(o);
      }

    }).start();

    configureAutoSelector();
  }

  private void configureButtonBindings() {
    // align turret
    new JoystickButton(joystick2, XboxController.Button.kLeftBumper.value)
      .whileHeld(new AlignTurretDefault(turretSubsystem, limelightSubsystem));
    
    // brake turret during hang
    new JoystickButton(joystick1, XboxController.Button.kB.value)
      .whileHeld(new BrakeTurret(turretSubsystem));

      // any distance, interpolation
    new JoystickButton(joystick2, XboxController.Button.kY.value)
      .whileHeld(new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem));

    // edge of tarmac
    new JoystickButton(joystick2, XboxController.Button.kX.value)
    .whileHeld(new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 7900, 9900, 0.04874, 0.049));

    // lower hub shot, consistent
    new JoystickButton(joystick2, XboxController.Button.kA.value)
      .whileHeld(new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 3500, 5450, 0.042, 0.0475));

      // coast arms during hanging
    new JoystickButton(joystick2, XboxController.Button.kRightBumper.value)
      .whileHeld(new CoastArms(armsSubsystem));

    new JoystickButton(joystick2, XboxController.Button.kStart.value)
    .whileHeld(new SmartLoad(feederSubsystem, loaderSubsystem, beamBreakSubsystem, ledSubsystem));
 
    // toggle to maintain elevator position
    new DoubleButton(
      new JoystickButton(joystick2, XboxController.Button.kA.value),
      new JoystickButton(joystick2, XboxController.Button.kB.value))
      .toggleWhenPressed(new MaintainElevatorPosition(elevatorSubsystem));
  }

  // private void smartDashboardBindings() {}

  private void configureAutoSelector() {
    // autonChooser.setDefaultOption("Preload Default", oneBallAuto);
    // autonChooser.addOption("2 Ball Exact Velocities", twoBallAutoExactVelocities);
    // autonChooser.addOption("2 Ball Interpolation", twoBallAutoInterpolation);
    // autonChooser.addOption("3 Ball Exact Velocities", threeBallAutoExactVelocities);
    // autonChooser.addOption("3 Ball Interpolation", threeBallAutoInterpolation);
    // SmartDashboard.putData(autonChooser);
  }

  public Command getAutonomousCommand() {
    // return oneBallAutoWait;

    // return oneBallAuto;
    // return twoBallAutoExactVelocities;
    //return cccTwoBall;
     return cccTwoBallCopy;
    // return twoBallAutoInterpoloation;
    // return threeBallAutoExactVelocities;
    // return threeBallAutoInterpolation;
    // return threeBallAuto;

    // return autonChooser.getSelected();
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    intakeSolenoidSubsystem.setDefaultCommand(deployIntakeWithJoystick);
    intakeSubsystem.setDefaultCommand(runIntakeAndLoaderWithJoystick);
    loaderSubsystem.setDefaultCommand(runIntakeAndLoaderWithJoystick); 
    feederSubsystem.setDefaultCommand(runFeederWithJoystick);
    turretSubsystem.setDefaultCommand(runTurretWithJoystick);
    elevatorSubsystem.setDefaultCommand(runElevatorWithJoystick);
    armsSubsystem.setDefaultCommand(runArmsWithJoystick);
    ledSubsystem.setDefaultCommand(setLEDColorWithJoystick);
  }
}

