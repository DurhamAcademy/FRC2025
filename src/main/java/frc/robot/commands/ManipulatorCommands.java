package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Manipulator;

public class ManipulatorCommands {
    public static Command runManipulator(Manipulator manipulator, double volts) {
        return Commands.runOnce(
                () -> {
                    manipulator.setVoltage(volts);
                },
                manipulator);
    }

    public static Command stopManipulator(Manipulator manipulator) {
        return Commands.runOnce(
                () -> {
                    manipulator.setVoltage(0);
                },
                manipulator);
    }

    public static Command eject(Manipulator manipulator) {
        return runManipulator(manipulator, 1).repeatedly();
    }

    public static Command eject(Manipulator manipulator, double volts) {
        return runManipulator(manipulator, volts).repeatedly();
    }

    public static Command algaeIntake(Manipulator manipulator) {
        return runManipulator(manipulator, -2.0).repeatedly();
    }

    public static Command intakeCoral(Manipulator manipulator) {
        return sequence(
                runManipulator(manipulator, 1.5).repeatedly()
                        .until(manipulator::beamBroken), // run until coral starts to enter
                runManipulator(manipulator, 0.75).repeatedly()
                        .until(() -> !manipulator.beamBroken()), // continue until too far
                runManipulator(manipulator, -0.6).repeatedly().until(manipulator::beamBroken),
                runManipulator(manipulator, 0.3).repeatedly().until(() -> !manipulator.beamBroken()),
                stopManipulator(manipulator));
    }
}
