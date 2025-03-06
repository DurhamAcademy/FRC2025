package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;

public class IntakeCommands {
    public static Command intakeCoral(Intake intake, Manipulator manipulator) {
        return parallel(
                        IntakeCommands.runIntake(intake)
                                .repeatedly()
                                .until(manipulator::beamBroken)
                                .andThen(IntakeCommands.stopIntake(intake)),
                        ManipulatorCommands.intakeCoral(manipulator))
                .andThen(
                        IntakeCommands.stopIntake(intake),
                        ManipulatorCommands.runManipulator(manipulator, 0));
    }

    public static Command runIntakeForCoral(Intake intake) {
        return sequence(runIntake(intake).onlyWhile(() -> !intake.getBeamBroken()));
    }

    public static Command runIntake(Intake intake) {
        return Commands.runOnce(() -> intake.setVoltage(3.0));
    }

    public static Command stopIntake(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setVoltage(0.0);
                },
                intake);
    }
}
