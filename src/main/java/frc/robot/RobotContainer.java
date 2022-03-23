// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.arms.ArmsSubsystem;
import frc.robot.subsystems.arms.RunArmsWithJoystick;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.MaintainElevatorPosition;
//import frc.robot.subsystems.elevator.MaintainElevatorPositionP;
import frc.robot.subsystems.elevator.RunElevatorWithJoystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController joystick1 = new XboxController(0);
  private final XboxController joystick2 = new XboxController(1);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmsSubsystem armsSubsystem = new ArmsSubsystem();
  private final RunElevatorWithJoystick runElevatorWithJoystick = new RunElevatorWithJoystick(elevatorSubsystem, joystick1);
  private final RunArmsWithJoystick runArmsWithJoystick = new RunArmsWithJoystick(armsSubsystem, joystick2);
  
  public RobotContainer() {
    configureButtonBindings();
  }
  /*
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick1, XboxController.Button.kB.value).toggleWhenPressed(new MaintainElevatorPosition(elevatorSubsystem));
    //new JoystickButton(joystick1, XboxController.Button.kX.value).toggleWhenPressed(new MaintainElevatorPositionP(elevatorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */

  // uncomment when u need to use this
  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

  // set default commands here
  public void setDefaultCommands(){
    elevatorSubsystem.setDefaultCommand(runElevatorWithJoystick);
    // armsSubsystem.setDefaultCommand(runArmsWithJoystick);
  }
}
