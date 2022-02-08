// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final XboxController joystick = new XboxController(0);
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // CameraServer setup
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
    // Configure the button bindings
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
    
  }
}
