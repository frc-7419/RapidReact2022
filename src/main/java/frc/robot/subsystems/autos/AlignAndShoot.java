// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeeder;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndShoot extends ParallelCommandGroup {
  /** Creates a new AllignAndShoot. */
  public AlignAndShoot(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, LoaderSubsystem loaderSubsystem, FeederSubsystem feederSubsystem, double topVelocity, double bottomVelocity) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    new AlignTurretDefault(turretSubsystem, limelightSubsystem);
    new GetToTargetVelocity(shooterSubsystem, topVelocity, bottomVelocity);
    new RunLoader(loaderSubsystem, 1);
    new RunFeeder(feederSubsystem, 1);

  }
}
