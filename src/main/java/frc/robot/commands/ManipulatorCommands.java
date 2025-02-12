package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Manipulator;

public class ManipulatorCommands {
    public static Command runManipulator(Manipulator manipulator, double power) {
        return Commands.run(
                () -> {
                    manipulator.setPower(power);
                },
                manipulator);
    }

    public static Command eject(Manipulator manipulator) {
        return runManipulator(manipulator, -.5);
    }

    public static Command intake(Manipulator manipulator) {
        return runManipulator(manipulator, .5);
    }

    public static Command safeIntake(Manipulator manipulator) {
        return intake(manipulator).onlyWhile(manipulator.beamBroken());
    }
}
