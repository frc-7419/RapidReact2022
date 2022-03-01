// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootThenMoveAway extends SequentialCommandGroup {
  /** Creates a new ShootThenMoveAway. */
  public ShootThenMoveAway(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, TurretSubsystem turretSubsystem, FeederSubsystem feederSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -30, PIDConstants.GyrokP30, PIDConstants.GyrokI30, PIDConstants.GyrokD30), new AlignAndShoot(turretSubsystem, limelightSubsystem, shooterSubsystem, feederSubsystem),new StraightWithMotionMagic(driveBaseSubsystem, -80.88));
  }
}
