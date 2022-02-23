// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transferWheel.TransferWheelSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(LoaderSubsystem loaderSubsystem, TransferWheelSubsystem transferWheelSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(new ParallelCommandGroup(new RunLoader())
  }
}
