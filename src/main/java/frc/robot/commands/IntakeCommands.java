package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;

public class IntakeCommands {
    public static Command retryStuckIntake(Intake intake, Manipulator manipulator) {
        return parallel(
                        ManipulatorCommands.runManipulator(manipulator, 0.0),
                        runIntake(intake, -1.0))
                .withTimeout(0.1)
                .andThen(fullCoralIntakeSequence(intake, manipulator));
    }

    public static Command pullCoralThroughIntake(Intake intake, Manipulator manipulator) {
        return parallel(
                        IntakeCommands.runIntake(intake, 3.0),
                        ManipulatorCommands.runManipulator(manipulator, 1.5))
                .until(manipulator::beamBroken)
                .andThen(IntakeCommands.stopIntake(intake));
    }

    public static Command fullCoralIntakeSequence(Intake intake, Manipulator manipulator) {
        return pullCoralThroughIntake(intake, manipulator)
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
