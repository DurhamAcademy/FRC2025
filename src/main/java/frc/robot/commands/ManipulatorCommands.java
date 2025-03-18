package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Manipulator;

public class ManipulatorCommands {
    public double secondForwardVolts = 0.75;
    public double backVolts = -0.6;
    public double thirdForwardVolts = 0.3;

    // sets voltage to manipulator
    public static Command runManipulator(Manipulator manipulator, double volts) {
        return Commands.run(
                () -> {
                    manipulator.setVoltage(volts);
                },
                manipulator);
    }

    // stops the manipulator not sure about brake or idle, assume idle because that is the brake
    // mode
    public static Command stopManipulator(Manipulator manipulator) {
        return Commands.runOnce(
                () -> {
                    manipulator.setVoltage(0);
                },
                manipulator);
    }

    // eject at a low voltage
    public static Command eject(Manipulator manipulator) {
        return runManipulator(manipulator, 1);
    }

    // eject at a certain voltage could be useful if net is slightly off
    public static Command eject(Manipulator manipulator, double volts) {
        return runManipulator(manipulator, volts);
    }

    // intakes algae
    public static Command algaeIntake(Manipulator manipulator) {
        return runManipulator(manipulator, -2.0);
    }

    // intakes at a very high voltage
    public static Command forceIntake(Manipulator manipulator) {
        return runManipulator(manipulator, 9);
    }

    // puts the coral in the correct spot in the manipulator
    public static Command intakeCoral(Manipulator manipulator) {
        return sequence(
                runManipulator(manipulator, 1.5)
                        .until(manipulator::beamBroken), // run until coral starts to enter
                runManipulator(manipulator, 0.75)
                        .until(() -> !manipulator.beamBroken()), // continue until too far
                runManipulator(manipulator, -0.6).until(manipulator::beamBroken),
                runManipulator(manipulator, 0.3).until(() -> !manipulator.beamBroken()),
                runManipulator(manipulator, 0));
    }
}
