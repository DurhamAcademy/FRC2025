package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
    private final ManipulatorIO io;
    private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
    private final SimpleMotorFeedforward ffModel;
    private final ProfiledPIDController pidController;

    public Manipulator(ManipulatorIO io) {
        this.io = io;
        // this is not important for right now, but just added this real quick in case we ever want
        // to set it to a certain speed for barge or whatnot
        switch (Constants.currentMode) {
            case REAL:
                // FIXME: characterize real robot
            case REPLAY:
                ffModel =
                        new SimpleMotorFeedforward(
                                ManipulatorConstants.manipulatorKs,
                                ManipulatorConstants.manipulatorKv);
                pidController =
                        new ProfiledPIDController(
                                ManipulatorConstants.manipulatorKp,
                                ManipulatorConstants.manipulatorKi,
                                ManipulatorConstants.manipulatorKd,
                                new TrapezoidProfile.Constraints(
                                        ManipulatorConstants.maxVelocity,
                                        ManipulatorConstants.maxAcceleration)); // tune velocity and
                // acceleration
                break;
            case SIM:
                ffModel =
                        new SimpleMotorFeedforward(
                                ManipulatorConstants.manipulatorKs,
                                ManipulatorConstants.manipulatorKv);
                pidController =
                        new ProfiledPIDController(
                                ManipulatorConstants.manipulatorKp,
                                ManipulatorConstants.manipulatorKi,
                                ManipulatorConstants.manipulatorKd,
                                new TrapezoidProfile.Constraints(
                                        ManipulatorConstants.maxVelocity,
                                        ManipulatorConstants.maxAcceleration)); // tune velocity and
                // acceleration
                break;
            default:
                ffModel = new SimpleMotorFeedforward(0.0, 0.0);
                pidController =
                        new ProfiledPIDController(
                                0., 0., .0, new TrapezoidProfile.Constraints(0., 0.));
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Manipulator", inputs);
    }

    public void setVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    public void setPower(double power) {
        io.setRollerPercent(power);
    }

    @AutoLogOutput
    public BooleanSupplier beamBroken() {
        BooleanSupplier supplier = () -> inputs.beamObstructed;
        return supplier;
    }
}
