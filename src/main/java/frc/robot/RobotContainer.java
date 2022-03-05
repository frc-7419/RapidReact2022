// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arms.ArmsSubsystem;
import frc.robot.subsystems.arms.RunArmsWithJoystick;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.RunElevatorWithJoystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.arms.ArmsSubsystem;
import frc.robot.subsystems.arms.RunArmsWithJoystick;
import frc.robot.subsystems.pneumatics.CompressorSubsystem;
import frc.robot.subsystems.pneumatics.RunPneumaticsSystem;
import frc.robot.subsystems.pneumatics.RunSolenoid;
import frc.robot.subsystems.pneumatics.SolenoidForwardAndReverse;
import frc.robot.subsystems.pneumatics.SolenoidSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController joystick = new XboxController(0);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmsSubsystem armsSubsystem = new ArmsSubsystem();
  private final RunElevatorWithJoystick runElevatorWithLimitSwitch = new RunElevatorWithJoystick(elevatorSubsystem, joystick);
  private final RunArmsWithJoystick runArmsWithJoystick = new RunArmsWithJoystick(armsSubsystem, joystick);
  
  public RobotContainer() {
    setDefaultCommands();
  }
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
  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

  // set default commands here
  public void setDefaultCommands(){
    elevatorSubsystem.setDefaultCommand(runElevatorWithLimitSwitch);
    armsSubsystem.setDefaultCommand(runArmsWithJoystick);
  }
}
