package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {

    public static Command moveElevatorLevel(Elevator elevator, double targetLevel) {
        return Commands.run(
                () -> {
                    elevator.setElevatorTargetHeight(targetLevel);
                    elevator.setWristTargetAngle(0);
                },
                elevator);
    }

    public static Command moveElevator(Elevator elevator, double power) {
        return Commands.run(
                () -> {
                    elevator.setElevatorPower(power);
                },
                elevator);
    }

    public static Command zeroElevator(Elevator elevator) {
        return moveElevator(elevator, -0.1); // .onlyWhile(() -> !elevator.isZeroed());
    }
}
