// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import javax.management.InstanceAlreadyExistsException;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeeder;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.SetLEDColor;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.BrakeTurret;
import frc.robot.subsystems.turret.TurretSubsystem;


public class HoustonFourBall extends ParallelCommandGroup {

  public HoustonFourBall(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, IntakeSolenoidSubsystem intakeSolenoidSubsystem, IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem, DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, FeederSubsystem feederSubsystem, LEDSubsystem ledSubsystem) {
    addCommands(sequence(
      //deploy intake and move forward to intake ball
      new InstantCommand(intakeSolenoidSubsystem::actuateSolenoid, intakeSolenoidSubsystem),
      
      //run intake and loader in parallel
      parallel(
        new RunIntake(intakeSubsystem, 1),
        new RunLoader(loaderSubsystem, 0.6)
      ).deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, 46)), 

      //brake turret, turn 180
      new WaitCommand(0.25),
      new BrakeTurret(turretSubsystem).deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, 2, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180)).withTimeout(0.25),
      new WaitCommand(0.15),

      //align turret, get to target velo, shoot 2 balls
      parallel(
        new AlignTurretDefault(turretSubsystem, limelightSubsystem), 
        new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem)
      ).withTimeout(0.75),

      new AlignAndShootWithLimelight(turretSubsystem, limelightSubsystem, shooterSubsystem, loaderSubsystem, feederSubsystem).withTimeout(1.5),

      //brake turret, turn 180 to face the terminal
      new WaitCommand(0.25),
      new BrakeTurret(turretSubsystem).deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, 2, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180)).withTimeout(0.25),
      new WaitCommand(0.15),

      parallel(
        new RunIntake(intakeSubsystem, 1),
        new RunLoader(loaderSubsystem, 1)
      ).deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, 187)), //calculate value for going straight all the way to the terminal

      new WaitCommand(1),
      //will do turn, forward shoot other 2 balls forward 187 inches
      
      //turn and go back
      new BrakeTurret(turretSubsystem).deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, 2, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180)).withTimeout(0.25),
      new StraightWithMotionMagic(driveBaseSubsystem, 160),

      //align turret, get to target velo, shoot 2 balls
      parallel(
        new AlignTurretDefault(turretSubsystem, limelightSubsystem), 
        new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem)
      ).withTimeout(0.75),

      new AlignAndShootWithLimelight(turretSubsystem, limelightSubsystem, shooterSubsystem, loaderSubsystem, feederSubsystem).withTimeout(1.5),

      new InstantCommand(driveBaseSubsystem::coast, driveBaseSubsystem)


    ));
    addCommands(new SetLEDColor(ledSubsystem, limelightSubsystem));


  }
}

//Face the ball (this ball is our third shot in 3 ball auton) on the field
//Move forward intake ball, turn, shoot 2 balls (preloaded and this one)
//Turn to face the terminal, move forward, intake ball by terminal and human player ball
//Turn back to face hub, move forward, shoot both balls