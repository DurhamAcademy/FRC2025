package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommands {
    public static Command runIntake(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setVoltage(9.0);
                },
                intake);
    }

    public static Command safeRunIntake(Intake intake) {
        return runIntake(intake)
                .onlyWhile(() -> !intake.getBeamBroken()); // stop when beam is broken
    }

    public static Command rotateIntakeUp(Intake intake) {
        return new RunCommand(() -> intake.setTargetRotation(IntakeConstants.INTAKE_UP_ROTATION));
    }

    public static Command rotateIntakeDown(Intake intake) {
        return new RunCommand(
                () -> intake.setTargetRotation(IntakeConstants.INTAKE_RESTING_ROTATION));
    }

    public static Command stopIntake(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setVoltage(0.0);
                },
                intake);
    }
}
