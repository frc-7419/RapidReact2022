// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.beambreak.WaitUntilShot;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeeder;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

// test comment
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndShoot extends SequentialCommandGroup {
  /** Creates a new AlignAndShoot. */
  public AlignAndShoot(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, BeamBreakSubsystem beamBreakSubsystem, int cargoToShoot) {
    // align turret and get to velocity
    addCommands(parallel(new AlignTurret(turretSubsystem, limelightSubsystem), new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem)));
    // once velocity reached, aim, run transfer wheel, and maintain velocity
    addCommands(race( // race ends when first ends, should probably use withInterrupt but idk how
      parallel(new RunFeeder(feederSubsystem, 0.5), new AlignTurret(turretSubsystem, limelightSubsystem), new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem)).withTimeout(5),
      new WaitUntilShot(beamBreakSubsystem, cargoToShoot)
    ));
  }
}
