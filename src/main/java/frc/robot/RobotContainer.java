// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.drive.StraightWithMotionMagicOld;
import frc.robot.subsystems.gyro.GyroSubsystem;
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
  private final Dashboard dashboard = new Dashboard();
  
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final GyroSubsystem gyro = new GyroSubsystem();

  // instantiate your auto commands here
  // private final StraightWithMotionMagic straightWithMotionMagic = new StraightWithMotionMagic(driveBaseSubsystem, 12);
  private final StraightWithMotionMagicOld straightWithMotionMagicOld = new StraightWithMotionMagicOld(driveBaseSubsystem, 12);
  

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    smartDashboardBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick, XboxController.Button.kY.value).whenPressed(new StraightWithMotionMagicOld(driveBaseSubsystem, 12));
  }

  private void smartDashboardBindings() {
    SmartDashboard.putNumber("mmKp", PIDConstants.DriveBaseMotionMagickP);
    SmartDashboard.putNumber("mmKi", PIDConstants.DriveBaseMotionMagickI);
    SmartDashboard.putNumber("mmKd", PIDConstants.DriveBaseMotionMagickD);

    SmartDashboard.putNumber("mmSetpoint", 12);
  }

  // uncomment when u need to use this
  public Command getAutonomousCommand() {
    return straightWithMotionMagicOld;
  }

  // schedule default commands here
  public void setDefaultCommands() {
    
  }
}
