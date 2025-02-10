package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorLevel;

public class ElevatorCommands {

    public static Command moveElevatorLevel(Elevator elevator, ElevatorLevel level) {
        return Commands.run(
                () -> {
                    elevator.setElevatorTargetHeight(level.heightInches);
                    elevator.setWristTargetAngle(level.angleRadians);
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
        // TODO: Change back when testing is complete and update for using wrist
        return moveElevator(elevator, -0.1); // .onlyWhile(() -> !elevator.isZeroed());
    }
}
