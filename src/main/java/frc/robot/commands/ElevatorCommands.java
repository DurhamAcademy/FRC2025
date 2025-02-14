package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorLevel;

public class ElevatorCommands {
    // command to set the target height of the elevator subsystem
    public static Command setElevatorLevel(Elevator elevator, ElevatorLevel level) {
        return Commands.run(() -> elevator.setTargetHeight(level.heightInches), elevator);
    }

    public static Command setElevatorVoltage(Elevator elevator, double voltage) {
        return Commands.run(() -> elevator.setVoltage(voltage), elevator);
    }

    public static Command zeroElevator(Elevator elevator) {
        return Commands.run(() -> setElevatorVoltage(elevator, -2).until(elevator::isZeroed));
    }
}
