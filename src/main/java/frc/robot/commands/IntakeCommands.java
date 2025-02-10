package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    public static Command runIntake(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setVoltage(9.0);
                },
                intake);
    }
}
