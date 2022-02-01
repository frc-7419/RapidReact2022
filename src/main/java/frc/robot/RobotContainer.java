package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.RunLoader;
import frc.robot.subsystems.colorSensor.RevColorDistanceSub;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import edu.wpi.first.wpilibj2.command.button.*;

public class RobotContainer {

  private final DriveBaseSubsystem driveBase = new DriveBaseSubsystem();
  private final ShooterSub shooter = new ShooterSub();
  private final XboxController joystick = new XboxController(0);
  private final LoaderSub loader = new LoaderSub();
  private final IntakeSub intake = new IntakeSub();
  private final RevolverSub revolver = new RevolverSub();
  private final ButtonBoard buttonBoard = new ButtonBoard();
  private final RevColorDistanceSub colorSensor = new RevColorDistanceSub();

  private final ArcadeDrive arcade = new ArcadeDrive(joystick, driveBase, 
  PowerConstants.DriveBaseLeftStraight.val, PowerConstants.DriveBaseRightTurn.val, 
  PowerConstants.DriveBaseRightStraight.val, PowerConstants.DriveBaseLeftTurn.val);

  private final IntakeDefault intakeDefault = new IntakeDefault(intake, joystick);
  private final RevolveWithIntake revolverDefault = new RevolveWithIntake(revolver, joystick);

  private BooleanSupplier bsLeftTrig = () -> Math.abs(joystick.getLeftTriggerAxis()) > .05;
  private Trigger xboxLeftTrigger = new Trigger(bsLeftTrig);

  private BooleanSupplier bsExternalRightJoystick = () -> buttonBoard.getJoystickX() == 1;
  private Trigger externalRightJoystick = new Trigger(bsExternalRightJoystick);

  private BooleanSupplier bsExternalLeftJoystick = () -> buttonBoard.getJoystickX() == -1;
  private Trigger externalLeftJoystick = new Trigger(bsExternalLeftJoystick);

  private BooleanSupplier bsExternalUpJoystick = () -> buttonBoard.getJoystickY() == 1;
  private Trigger externalUpJoystick = new Trigger(bsExternalUpJoystick);

  private BooleanSupplier bsExternalDownJoystick = () -> buttonBoard.getJoystickY() == -1;
  private Trigger externalDownJoystick = new Trigger(bsExternalDownJoystick);
 

  public RobotContainer() {
    manualButtonBindings();
    buttonBoardBindings();
  }

  private void manualButtonBindings(){ // for johann

    // X button
    new JoystickButton(joystick, XboxController.Button.kX.value)
    .whileHeld(new GetToTargetVelocity(shooter, PowerConstants.ShooterJohann.val));


    // new JoystickButton(joystick, PaddedXbox.F310Map.kGamepadButtonX.value)
    // .whileHeld(new GetToTargetVelocity(shooter, PowerConstants.ShooterJohann.val));

    // L Shoulder
    new JoystickButton(joystick, XboxController.Button.kLeftBumper.value)
    .whileHeld(new RunRevolver(revolver, PowerConstants.RevolverJohann.val, false)); 
    
    // R Shoulder
    new JoystickButton(joystick, XboxController.Button.kRightBumper.value)
    .whileHeld(new RunRevolver(revolver, PowerConstants.RevolverJohann.val, true)); 


    new POVButton(joystick, 0).whileHeld(new RunLoader(loader, PowerConstants.LoaderJohann.val, true)); 
    new POVButton(joystick, 180).whileHeld(new RunLoader(loader, PowerConstants.LoaderJohann.val, false));

    new POVButton(joystick, 90).whenPressed(new RevolverToTape(colorSensor, revolver)); 
  }

  public void buttonBoardBindings(){

    // 1: revolver to tape
    new JoystickButton(buttonBoard, 1)
    .whenPressed(new RevolverToTape(colorSensor, revolver).withTimeout(3));
    

    // 2: Revolver to Speed
    new JoystickButton(buttonBoard, 2)
    .whileHeld(new GetToTargetVelocity(shooter, PowerConstants.ShooterShotsButton.val));
    
    // new JoystickButton(buttonBoard, 2)
    // .whileHeld(new RunShooter(  shooter, loader, revolver, PowerConstants.ShooterShotsButton.val, 
    //                             PowerConstants.RevolverShotsButton.val));

    // 3: 5419 SHOTS
    
    new JoystickButton(buttonBoard, 3)
    .whileHeld(new RunLoader(loader, PowerConstants.LoaderShotsButton.val, true));

    // new JoystickButton(buttonBoard, 3)
    // .whileHeld(new RunShooter(  shooter, loader, revolver, PowerConstants.Shooter5419Shots.val, 
    //                             PowerConstants.Revolver5419Shots.val));

    

    // 4: henry's off the wall thing at 9 inches <--- wrong 
    // 4: rotate revolver and gets to target velocity

    new JoystickButton(buttonBoard, 4)
    .whileHeld(new GetToTargetVelocity(shooter, PowerConstants.ShooterShotsButtonLong.val));

    // new JoystickButton(buttonBoard, 4)
    // .whenPressed(new RevolverToTape(colorSensor, revolver).withTimeout(3));
    // new JoystickButton(buttonBoard, 4)
    // .whileHeld(new GetToTargetVelocity(shooter, PowerConstants.Shooter5419Shots.val));
    
    // 5: cp down & no spin
    // new JoystickButton(buttonBoard, 5)
    // .whileHeld(new RaiseCpMech(cpMech, .25, true));

    // 6: cp up, spin after a delay
    // new JoystickButton(buttonBoard, 6)
    // .whileHeld(new UpThenSpin(cpMech, .25, false, 2, .25));

    // // 5: go back x inches 
    // new JoystickButton(buttonBoard, 5)
    // .whenPressed(new StraightWithMotionMagic(driveBase, 12));

    // 12: climb up at .9
    // new JoystickButton(buttonBoard, 12)
    // .whileHeld(new RunClimber(climber, -PowerConstants.ClimberOperator.val, false));

    // run revolver on external joystick x axis
    externalRightJoystick.whileActiveOnce(new RunRevolver(revolver, PowerConstants.RevolverButtonBoard.val, true));
    externalLeftJoystick.whileActiveOnce(new RunRevolver(revolver, PowerConstants.RevolverButtonBoard.val, false));

    // run intake on external joystick y axis
    externalDownJoystick.whileActiveOnce(new RunIntake(intake, joystick, PowerConstants.IntakeJohannGround.val));
    externalUpJoystick.whileActiveOnce(new RunIntake(intake, joystick, PowerConstants.IntakeJohannPlayerStation.val));
  }

  public Command getDefaultCommand(){return arcade;}
  // public Command getLimelightTest(){return turnToTx;}
  
  public void setDefaultCommands(){
    // revolver.setDefaultCommand(revolverDefault);
    driveBase.setDefaultCommand(arcade);
    intake.setDefaultCommand(intakeDefault);
  }

    public Command getAutonomousCommand() {
      return new WaitCommand(0);
    }
  
}
