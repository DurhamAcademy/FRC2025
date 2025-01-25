package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorLevel;

public class ElevatorCommands {

    public static Command moveElevator(Elevator elevator, ElevatorLevel targetLevel) {
        return Commands.run(
                () -> {
                    elevator.setTargetHeight(targetLevel.heightInches);
                });
    }
}
