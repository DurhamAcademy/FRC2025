package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

public class DriveCommandsInterface {
    /**
     * Class that holds command and ready supplier
     */
    record CommandAndReadySupplier(Command command, BooleanSupplier readySupplier) {
        /**
         * Default constructor for CommandAndReadySupplier class
         * @param command       the command
         * @param readySupplier the function/lambda that will return a boolean
         */
        public CommandAndReadySupplier {}
    }
}
