package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorLevel;

public class ElevatorCommands {
    // command to set the target height of the elevator subsystem
    public static Command setElevatorLevel(Elevator elevator, ElevatorLevel level) {
        return Commands.runOnce(
                () -> {
                    System.out.println("Elevator Level: " + level);
                    elevator.setElevatorTargetHeight(level.heightInches);
                    elevator.setWristTargetAngle(level.angleRadians);
                },
                elevator);
    }

    public static Command setElevatorVoltage(Elevator elevator, double voltage) {
        return Commands.run(() -> elevator.setVoltage(voltage), elevator);
    }

    public static Command zeroElevator(Elevator elevator) {
        return setElevatorLevel(elevator, ElevatorLevel.ZERO)
                .until(() -> Math.abs(0 - elevator.getElevatorHeight()) < 1)
                .andThen(setElevatorVoltage(elevator, -.1).until(elevator::isZeroed));
    }
}
