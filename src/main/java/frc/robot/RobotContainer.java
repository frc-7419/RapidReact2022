// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.limitswitch.LimitswitchSubsystem;
import frc.robot.subsystems.limitswitch.RunElevatorWithLimitSwitch;
import frc.robot.subsystems.talon.RunMotorWithLimitSwitch;
import frc.robot.subsystems.talon.TalonSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController joystick = new XboxController(0);
  private final LimitswitchSubsystem limitSwitchSubsystem = new LimitswitchSubsystem();
  private final TalonSubsystem talonSubsystem = new TalonSubsystem();
  private final RunMotorWithLimitSwitch runMotorWithLimitSwitch = new RunMotorWithLimitSwitch(limitSwitchSubsystem, talonSubsystem);
  private final RunMotorWithLimitSwitch runElevatorWithLimitSwitch = new RunMotorWithLimitSwitch(limitSwitchSubsystem, talonSubsystem);
  /*
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
    talonSubsystem.setDefaultCommand(runMotorWithLimitSwitch);
  }
}
