// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.Coast;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeeder;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.intake.DeployIntake;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.BrakeTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SvrThreeBall extends SequentialCommandGroup {

  public SvrThreeBall(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, LoaderSubsystem loaderSubsystem, FeederSubsystem feederSubsystem, DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, IntakeSubsystem intakeSubsystem, IntakeSolenoidSubsystem intakeSolenoidSubsystem) {
    addCommands(
      // gttv and align turret
      parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocity(shooterSubsystem, 37, 30))
        .withInterrupt(() -> shooterSubsystem.bothOnTarget()).withTimeout(0.75), // gttv while aligning turret

      // shoot preload
      parallel(
          new AlignTurretDefault(turretSubsystem, limelightSubsystem),
          new GetToTargetVelocity(shooterSubsystem, 37, 30),
          new RunFeeder(feederSubsystem, 1),
          new RunLoader(loaderSubsystem, 1)
      ).withTimeout(1.5), // tune time

      // retract turret
      new InstantCommand(intakeSolenoidSubsystem::retractSolenoid, intakeSolenoidSubsystem),

      // turn 180 while braking turret
      new BrakeTurret(turretSubsystem)
          .deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180)),

      new WaitCommand(0.25),  
            
      // deploy intake
      new InstantCommand(intakeSolenoidSubsystem::actuateSolenoid, intakeSolenoidSubsystem),

      // move forward and running intake + loader
      parallel(
          new RunIntake(intakeSubsystem, 1),
          new RunLoader(loaderSubsystem, 0.6)
      ).deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, 50)),
      
      new WaitCommand(0.25),

      // retract intake
      new InstantCommand(intakeSolenoidSubsystem::retractSolenoid, intakeSolenoidSubsystem),

      // turn 180 while braking turret
      new BrakeTurret(turretSubsystem)
          .deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 115, PIDConstants.GyrokP115, PIDConstants.GyrokI115, PIDConstants.GyrokD115)),

      new WaitCommand(0.25),

      // deploy intake
      new InstantCommand(intakeSolenoidSubsystem::actuateSolenoid, intakeSolenoidSubsystem),

      // move forward 116.17 and gttv
      new GetToTargetVelocity(shooterSubsystem, 45.5, 43)
        .deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, 116.17)),

      parallel(new GetToTargetVelocity(shooterSubsystem, 45.5, 43), new AlignTurretDefault(turretSubsystem, limelightSubsystem))
        .deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, -50)),
      
      // shoot ball
      parallel(
          new AlignTurretDefault(turretSubsystem, limelightSubsystem),
          new GetToTargetVelocity(shooterSubsystem, 37, 30),
          new RunFeeder(feederSubsystem, 1),
          new RunLoader(loaderSubsystem, 1)
      ).withTimeout(1.5), // tune time
      
      new InstantCommand(driveBaseSubsystem::coast, driveBaseSubsystem)
    );
  }
}

// shoot preload
// turn 180
// forward to ball while intake (parallel)
// turn toward third ball
// forward to third ball while intake (parallel)
// turn toward hub
// shoot both
