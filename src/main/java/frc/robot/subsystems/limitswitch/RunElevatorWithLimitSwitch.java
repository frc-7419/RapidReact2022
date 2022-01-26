// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limitswitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunElevatorWithLimitSwitch extends CommandBase {
  /** Creates a new RunElevatorWithLimitSwitch. */
  private ElevatorSubsystem elevatorSubsystem;
  private XboxController joystick;

  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  public RunElevatorWithLimitSwitch(ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
    topLimitSwitch = new DigitalInput(1);
    bottomLimitSwitch = new DigitalInput(0);
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Working?", "reached execute command");
    if (!topLimitSwitch.get() && joystick.getRightY() > 0) {
      
      elevatorSubsystem.setPower(0);
    } else if (!bottomLimitSwitch.get() && joystick.getRightY() < 0) {
      
      elevatorSubsystem.setPower(0);
    } else {
      elevatorSubsystem.setPower(0.4 * joystick.getRightY());
      
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
