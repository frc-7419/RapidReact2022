// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.loader;

import com.team7419.InterpolatedTreeMap;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SmartLoad extends CommandBase {
  /** Creates a new SmartShoot. */
  private FeederSubsystem feederSubsystem;
  private LoaderSubsystem loaderSubsystem;
  private BeamBreakSubsystem beamBreakSubsystem;
  private LEDSubsystem ledSubsystem;

  public SmartLoad(FeederSubsystem feederSubsystem, LoaderSubsystem loaderSubsystem, BeamBreakSubsystem beamBreakSubsystem, LEDSubsystem ledSubsystem) {
    this.feederSubsystem = feederSubsystem;
    this.loaderSubsystem = loaderSubsystem;
    this.beamBreakSubsystem = beamBreakSubsystem;
    this.ledSubsystem = ledSubsystem;
    addRequirements(feederSubsystem, loaderSubsystem, beamBreakSubsystem, ledSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if beam break broken, run feeder and loader in reverse until beam break is not broken
    if (beamBreakSubsystem.getBeamBreakActivated()) {
        feederSubsystem.setPower(-0.5);
        loaderSubsystem.setPower(-0.5);
    } else {
        feederSubsystem.setPower(0);
        loaderSubsystem.setPower(0);
        ledSubsystem.setLEDColor(0, 0, 255);
    }
  }


  // Called once the command ends or is interrupted.


  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
