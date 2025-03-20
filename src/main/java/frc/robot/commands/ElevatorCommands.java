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

    public static Command zeroElevatorForCoral(Elevator elevator){
        return setElevatorLevel(elevator, ElevatorLevel.INTAKE)
                .andThen(
                        Commands.waitUntil(elevator::elevatorIsAtSetpoint).withTimeout(5))
                .andThen(setElevatorVoltage(elevator, -.5).until(elevator::isZeroed));
    }

    public static Command zeroElevatorForAlgae(Elevator elevator) {
        return setElevatorLevel(elevator, ElevatorLevel.ZERO)
                .andThen(
                        Commands.waitUntil(elevator::elevatorIsAtSetpoint).withTimeout(5))
                .andThen(setElevatorVoltage(elevator, -.5).until(elevator::isZeroed));
    }
}
