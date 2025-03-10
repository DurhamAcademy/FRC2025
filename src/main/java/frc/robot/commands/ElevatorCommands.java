package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorLevel;

public class ElevatorCommands {
    // command to set the target height setpoints of the elevator subsystem
    public static Command setElevatorLevel(Elevator elevator, ElevatorLevel level) {
        return Commands.runOnce(
                () -> {
                    elevator.setElevatorTargetHeight(level.heightInches);
                    elevator.setWristTargetAngle(level.angleRadians);
                },
                elevator);
    }

    // command to run voltage on the elevator motors
    public static Command setElevatorVoltage(Elevator elevator, double voltage) {
        return Commands.run(() -> elevator.setVoltage(voltage), elevator);
    }

    // command to run the elevator until it hits the limit switch
    public static Command zeroElevator(Elevator elevator) {
        return setElevatorLevel(elevator, ElevatorLevel.ZERO)
                .andThen(
                        Commands.waitUntil(() -> elevator.getElevatorHeight() < 1)
                                .withTimeout(5))
                .andThen(setElevatorVoltage(elevator, -.5).until(elevator::isZeroed));
    }
}
