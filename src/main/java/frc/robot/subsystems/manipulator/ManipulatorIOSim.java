package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ManipulatorIOSim implements ManipulatorIO {

    private final FlywheelSim rollerSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getNEO(2), 3, 1.0 / ManipulatorConstants.manipulatorGearRatio),
                    DCMotor.getNEO(2));
    // TODO: change the jKg meters squared in this once we get cad and everything
    private double rollerVoltage = 0.0;
    private Double timestamp = null;

    public ManipulatorIOSim() {
        setArmVoltage(0.0);
        setRollerPercent(0.0);
    }

    public void updateInputs(ManipulatorIOInputs inputs) {
        var ct = Timer.getFPGATimestamp();
        var dt = (timestamp == null) ? .02 : ct - timestamp;

        inputs.rollerRCurrentAmps = new double[] {rollerSim.getCurrentDrawAmps()};
        inputs.rollerRAppliedVolts = rollerVoltage;
        inputs.rollerRVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();

        inputs.rollerLCurrentAmps = new double[] {rollerSim.getCurrentDrawAmps()};
        inputs.rollerLAppliedVolts = rollerVoltage;
        inputs.rollerLVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
        rollerSim.update(dt);
        timestamp = ct;
    }

    /** Set intake wheel voltage. */
    public void setRollerPercent(double percent) {
        rollerVoltage = percent * 12.0;
        rollerSim.setInputVoltage(rollerVoltage);
    }

    public void setRollerVoltage(double volts) {
        rollerSim.setInputVoltage(volts);
    }
}
