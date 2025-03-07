package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;

public class IntakeCommands {
    public static Command intakeCoral(Intake intake, Manipulator manipulator) {
        double maxAttemptTime = 3.0;
        boolean[] timedOut = {false}; // Use an array to allow mutation inside lambdas

        Command intakeSequence =
                parallel(
                        IntakeCommands.runIntake(intake, 2.0)
                                .repeatedly()
                                .until(manipulator::beamBroken)
                                .andThen(IntakeCommands.stopIntake(intake)),
                        ManipulatorCommands.intakeCoral(manipulator))
                        .withTimeout(maxAttemptTime)
                        .handleInterrupt(() -> timedOut[0] = true); // mark timeout on interrupt

        return new SequentialCommandGroup(
                new InstantCommand(() -> timedOut[0] = false), // reset flag before starting
                intakeSequence,
                new ConditionalCommand( // sequence to run backwards
                        new SequentialCommandGroup(
                                // reverse until unstuck
                                IntakeCommands.runIntake(intake, -4.0).until(() -> !intake.getBeamBroken()),
                                IntakeCommands.stopIntake(intake),
                                intakeSequence), // retry intake sequence
                        Commands.none(), // continue if not stuck
                        () -> timedOut[0]), // only retry if timeout happened
                IntakeCommands.stopIntake(intake),
                ManipulatorCommands.runManipulator(manipulator, 0));
    }

    public static Command runIntake(Intake intake, double volts) {
        return Commands.runOnce(() -> intake.setVoltage(volts));
    }

    public static Command stopIntake(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setVoltage(0.0);
                },
                intake);
    }
}
