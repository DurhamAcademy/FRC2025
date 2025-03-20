package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;

public class IntakeCommands {
    public static Command retryStuckIntake(
            Intake intake, Manipulator manipulator, Elevator elevator) {
        return parallel(
                        ManipulatorCommands.runManipulator(manipulator, 0.0),
                        runIntake(intake, -2.0))
                .withTimeout(0.1)
                .andThen(fullCoralIntakeSequence(intake, manipulator, elevator));
    }

    public static Command pullCoralThroughIntake(
            Intake intake, Manipulator manipulator, Elevator elevator) {
        return either(
                IntakeCommands.runIntake(intake, .3)
                        .until(intake::getBeamBroken),
                parallel(
                                IntakeCommands.runIntake(intake, 3.0),
                                ManipulatorCommands.runManipulator(manipulator, 1.5))
                        .until(manipulator::beamBroken)
                        .andThen(IntakeCommands.stopIntake(intake)),
                () -> elevator.getElevatorHeight() < .5);
    }

    public static Command fullCoralIntakeSequence(
            Intake intake, Manipulator manipulator, Elevator elevator) {
        return pullCoralThroughIntake(intake, manipulator, elevator)
                .andThen(ManipulatorCommands.pullCoralIntoManipulator(manipulator))
                .andThen(ManipulatorCommands.coralIntakeRipple(manipulator))
                .andThen(
                        IntakeCommands.stopIntake(intake),
                        ManipulatorCommands.stopManipulator(manipulator));
    }

    public static Command runIntake(Intake intake, double voltage) {
        return Commands.runOnce(() -> intake.setVoltage(voltage), intake);
    }

    public static Command stopIntake(Intake intake) {
        return Commands.runOnce(
                () -> {
                    intake.setVoltage(0.0);
                },
                intake);
    }
}
