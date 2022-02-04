// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeCommandColorSensor.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakeCommandColorSensor.colorSensor.ColorSensorSubsystem;

public class RunIntakeIfBlue extends CommandBase {
  private ColorSensorSubsystem colorSensorSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private double power;

  /** Creates a new RunIntakeWithColorSensor. */
  public RunIntakeIfBlue(IntakeSubsystem intakeSubsystem, ColorSensorSubsystem colorSensorSubsystem, double power) {
    this.intakeSubsystem = intakeSubsystem;
    this.colorSensorSubsystem = colorSensorSubsystem;
    this.power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    addRequirements(colorSensorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // assume alliance is blue
    double[] colors = colorSensorSubsystem.getRgb();
    if (colors[0] < colors[2]) {
      intakeSubsystem.setPower(power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
