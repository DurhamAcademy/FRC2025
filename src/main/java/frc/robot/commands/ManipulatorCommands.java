package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

public class ManipulatorCommands {
    public double secondForwardVolts = 0.75;
    public double backVolts = -0.6;
    public double thirdForwardVolts = 0.3;

    public static Command runManipulator(Manipulator manipulator, double volts) {
        return Commands.run(
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

    public static Command coralEject(Manipulator manipulator, Elevator elevator) {
        return normalCoralEject(manipulator);
        //double height = elevator.getElevatorHeight();
        //if (height >= Elevator.ElevatorLevel.L1.heightInches - .5
        //        && height <= Elevator.ElevatorLevel.L1.heightInches + .5) {
        //    return l1Eject(manipulator);
        //} else {
        //    return normalEject(manipulator);
        //}
    }

    public static Command algaeEject(Manipulator manipulator, Elevator elevator){
        double height = elevator.getElevatorHeight();
        if (height >= Elevator.ElevatorLevel.PROCESSOR.heightInches - .5
                && height <= Elevator.ElevatorLevel.PROCESSOR.heightInches + .5) {
            return processorEject(manipulator);
        } else {
            return netEject(manipulator);
        }
    }

    public static Command netEject(Manipulator manipulator) {
        return eject(manipulator, 4);
    }

    public static Command processorEject(Manipulator manipulator) {
        return runManipulator(manipulator, 1);
    }

    public static Command normalCoralEject(Manipulator manipulator) {
        return runManipulator(manipulator, 1);
    }

    public static Command eject(Manipulator manipulator, double volts) {
        return runManipulator(manipulator, volts);
    }

    public static Command algaeIntake(Manipulator manipulator) {
        return runManipulator(manipulator, -2.0);
    }

    public static Command forceIntake(Manipulator manipulator) {
        return runManipulator(manipulator, 9);
    }

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
