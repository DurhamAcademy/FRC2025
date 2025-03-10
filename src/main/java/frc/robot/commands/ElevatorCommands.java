package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorLevel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;

public class ElevatorCommands {
    // command to set the target height of the elevator subsystem
    public static Command setElevatorLevel(
            Elevator elevator, Intake intake, Manipulator manipulator, ElevatorLevel level) {
        return Commands.runOnce(
                () -> {
                    // only runs if the coral isn't still in the intake or manipulator beam breaks
                    // this avoids elevator the coral getting stuck in the middle of the elevator
                    if (!intake.getBeamBroken() && !manipulator.beamBroken()) {
                        elevator.setElevatorTargetHeight(level.heightInches);
                        elevator.setWristTargetAngle(level.angleRadians);
                    }
                },
                elevator);
    }

    public static Command setElevatorVoltage(Elevator elevator, double voltage) {
        return Commands.run(() -> elevator.setVoltage(voltage), elevator);
    }

    public static Command zeroElevator(Elevator elevator, Intake intake, Manipulator manipulator) {
        return setElevatorLevel(elevator, intake, manipulator, ElevatorLevel.ZERO)
                .andThen(
                        Commands.waitUntil(() -> Math.abs(0 - elevator.getElevatorHeight()) < 1)
                                .withTimeout(5))
                .andThen(setElevatorVoltage(elevator, -.5).until(elevator::isZeroed));
    }

    // TODO consider this
    /*
    public static Command zeroElevator(Elevator elevator) {
        return setElevatorLevel(elevator, ElevatorLevel.ZERO).repeatedly()
                .until(() -> Math.abs(0 - elevator.getElevatorHeight()) < 1).withTimeout(5.0)
                .andThen(setElevatorVoltage(elevator, -.1).until(elevator::isZeroed));
    }
     */
}
