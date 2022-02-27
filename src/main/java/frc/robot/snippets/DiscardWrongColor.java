// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.snippets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretPowerTime;
import frc.robot.subsystems.colorSensor.RevColorDistanceSub;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class DiscardWrongColor extends CommandBase {
  /** Creates a new DiscardWrongColor. */
  private TurretSubsystem turretSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private RevColorDistanceSub revColorDistanceSub;
  private String wrongColor = "blue";
  private Boolean ran = false;
  private double time = 0.1;
  private double startTime;
  private double power = 0.1;
  public DiscardWrongColor(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem,RevColorDistanceSub revColorDistanceSub) {
    this.turretSubsystem = turretSubsystem;
    this.revColorDistanceSub = revColorDistanceSub;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(turretSubsystem, shooterSubsystem, revColorDistanceSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (revColorDistanceSub.getColor() == wrongColor && !ran){

        SmartDashboard.putString("discard: ", "true");
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time){
            turretSubsystem.setPower(power);
        }
        shooterSubsystem.setBothPower(0.2);
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time){
            turretSubsystem.setPower(-power);
        }
        ran=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
