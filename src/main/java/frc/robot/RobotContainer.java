// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.colorSensor.ColorSensorSubsystem;
import frc.robot.subsystems.colorSensor.RevColorDistanceSub;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;

public class RobotContainer {
  private final XboxController joystick = new XboxController(0);
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  private final ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem();
  private final RevColorDistanceSub revColorDistanceSub = new RevColorDistanceSub();
  
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick, XboxController.Button.kRightBumper.value).toggleWhenPressed(new RunLoader(loaderSubsystem, joystick, 0.3));
  }

  public void setDefaultCommands() {
    
  }
}
