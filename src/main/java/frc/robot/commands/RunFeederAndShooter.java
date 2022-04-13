// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeederWithJoystick;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.loader.RunLoaderWithJoystick;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunFeederAndShooter extends ParallelCommandGroup {
  /** Creates a new RunLoaderAndShooter. */
  public RunFeederAndShooter(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, XboxController joystick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      parallel(
        new RunFeederWithJoystick(feederSubsystem, joystick, 1),
        new GetToTargetVelocity(shooterSubsystem, 10, 10)
      )
    );
  }
}