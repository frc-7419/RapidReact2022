package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.RunTalon;
import frc.robot.subsystems.TalonSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {
  private final TalonSubsystem talonSubsystem;
  private final RunTalon runTalon;
  private final XboxController joystick1;

  public RobotContainer() {
    joystick1 = new XboxController(0);
    talonSubsystem = new TalonSubsystem();
    runTalon = new RunTalon(talonSubsystem, joystick1);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */

  // uncomment when u need to use this
  // public Command getAutonomousCommand() {
  //   return autonomousCommand;
  // }

  // set default commands here
  public void setDefaultCommands(){
    talonSubsystem.setDefaultCommand(runTalon);
  }
}
