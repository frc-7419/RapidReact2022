// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.MaintainElevatorPosition;
import frc.robot.subsystems.elevator.SetElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimb extends ParallelCommandGroup {
  /** Creates a new AutoClimb. */
  public AutoClimb(ArmsSubsystem armsSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      sequence(
        // new SetElevatorPosition(elevatorSubsystem, -181363, 0.00005, 0).withTimeout(2),
        new SetElevatorPosition(elevatorSubsystem, 0, 0.00005, 0.0615),
        new MaintainElevatorPosition(elevatorSubsystem, 0, 0.00005, 0.0615)
        .deadlineWith(new SetArmPosition(armsSubsystem, 5.6429, 0.1, 0)).withTimeout(2),
        new SetElevatorPosition(elevatorSubsystem, -100000, 0.00005, 0.0615),
        new SetArmPosition(armsSubsystem, -1, 0.1, 0),
        new SetElevatorPosition(elevatorSubsystem, -161632, 0.000005, 0),
        new SetArmPosition(armsSubsystem, 1.8, 0.1, 0),
        new MaintainArmPosition(armsSubsystem, 1.8, 0.1, 0)
        .deadlineWith(new SetElevatorPosition(elevatorSubsystem, 0, 0.000005, 0.0615))
        // new SetElevatorPosition(elevatorSubsystem, 0, 0.00005, 0.0615)




      )
    );
  }
}
