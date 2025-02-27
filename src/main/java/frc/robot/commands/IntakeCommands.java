package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommands {
    public static Command runIntake(Intake intake) {
        return Commands.runEnd(
                () -> intake.setVoltage(1.0), // runs while active
                () -> intake.setVoltage(0.0), // stops on end
                intake);
    }

    public static Command safeRunIntake(Intake intake) {
        return runIntake(intake).onlyWhile(() -> !intake.getBeamBroken());
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
