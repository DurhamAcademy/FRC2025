package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Manipulator;

public class ManipulatorCommands {
    public static Command runManipulator(Manipulator manipulator, double volts) {
        return Commands.run(
                () -> {
                    manipulator.setVoltage(volts);
                },
                manipulator);
    }

    public static Command eject(Manipulator manipulator) {
        return runManipulator(manipulator, 9);
    }

    public static Command algaeIntake(Manipulator manipulator) {
        return runManipulator(manipulator, -2.0);
    }

    public static Command forceIntake(Manipulator manipulator) {
        return runManipulator(manipulator, 9);
    }

    /*public static Command humanPlayerIntake(Manipulator manipulator) {
        return sequence(
                        runManipulator(manipulator, 9).until(manipulator.beamBroken()),
                        runManipulator(manipulator, 9).withTimeout(.05),
                        runManipulator(manipulator, 9).onlyWhile(manipulator.beamBroken()))
                .andThen(runManipulator(manipulator, 0));
    }*/

    public static Command intakeCoral(Manipulator manipulator) {
        return sequence(
                runManipulator(manipulator, 5.0)
                        .until(manipulator::beamBroken), // run until coral starts to enter
                runManipulator(manipulator, 5.0)
                        .until(() -> !manipulator.beamBroken()), // continue until too far
                runManipulator(manipulator, -5.0)
                        .until(manipulator::beamBroken) // move back until coral breaks beam
                );
    }
}
