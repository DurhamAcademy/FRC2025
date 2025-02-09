package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ElevatorIOSim implements ElevatorIO {
    // TODO: make sure to change the values to correct !!
    private static final double ElevatorMass = 1.0;
    private static final double ElevatorGearRatio = 1.0;
    private static final double ElevatorMaxHeight = 1.0;
    private static final double ElevatorWheelRadius = 1.0;
    private static final double Measurementstdv1 =
            0.01; // i am just making this variable because the constructor needs it
    private static final double Measurementstdv2 = 0.0;
    private static final double ElevatorInitPos =
            0.0; // same with this, this must be measured and replaced

    // i think this is more hardware related, so probably should be in the elevator class?
    // hardware control code, will write the digital signals to the ports
    private final Encoder m_encoder =
            new Encoder(ElevatorConstants.rightElevatorCanId, ElevatorConstants.leftElevatorCanId);
    // also this is inaccruate
    private final PWMSparkMax m_motor = new PWMSparkMax(17); // change this and it works

    // ***Elevator motorport --> limit switch port? PWMsparkmax() rel freq nl***

    // effective for smoooth trajectories and travel, enforcing acceleration and velocity limits
    // more simulation-suitable class compared to normal PIDController
    private final ProfiledPIDController m_controller =
            new ProfiledPIDController(
                    ElevatorConstants.elevatorKp,
                    ElevatorConstants.elevatorKi,
                    ElevatorConstants.elevatorKd,
                    new TrapezoidProfile.Constraints(
                            ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration));

    // helper class that helps calculate required voltage, compensating for real life physics
    ElevatorFeedforward m_feedforward =
            new ElevatorFeedforward(
                    ElevatorConstants.elevatorKs,
                    ElevatorConstants.elevatorKg,
                    ElevatorConstants.elevatorKv,
                    ElevatorConstants.elevatorKa);

    // starting sim stuff (hopefully)
    // I am not sure if we are using falcon 500 motors, so someone must check on that
    // bru am i even doing this right
    private final ElevatorSim elevatorSim =
            new ElevatorSim(
                    LinearSystemId.createElevatorSystem(
                            DCMotor.getFalcon500(1),
                            ElevatorMass,
                            ElevatorWheelRadius,
                            ElevatorGearRatio),
                    DCMotor.getFalcon500(1),
                    ElevatorConstants.ZERO, // min height
                    ElevatorConstants.L4, // max height
                    true, // simulate gravity
                    ElevatorInitPos,
                    Measurementstdv1,
                    Measurementstdv2);

    // this code gets the values written by the motor control code and simulates it
    // not sure if i can use DutyCycleEncoderSim
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    private final PWMSim m_motorSim = new PWMSim(m_motor);

    // mechanism2d stuff
    // discard later
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_mech2d_root = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d =
            m_mech2d_root.append(
                    new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

    public void simulationPeriodic() {
        elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());
        elevatorSim.update(0.02);
        m_encoderSim.setDistance(elevatorSim.getPositionMeters());
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
}
