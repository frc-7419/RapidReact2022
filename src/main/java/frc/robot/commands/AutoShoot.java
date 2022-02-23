// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transferWheel.RunTransferWheel;
import frc.robot.subsystems.transferWheel.TransferWheelSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class AutoShoot extends SequentialCommandGroup {
  public AutoShoot(LoaderSubsystem loaderSubsystem, TransferWheelSubsystem transferWheelSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    addCommands(new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem).withInterrupt(shooterSubsystem::bothOnTarget));
    addCommands(new ParallelCommandGroup(new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem)), new RunLoader(loaderSubsystem, 0.5), new RunTransferWheel(transferWheelSubsystem, 1));
  }
}
