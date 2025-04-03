package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    public static final int stripLength = 110;
    public static final int candleLength = 8;

    public CANdle candle;

    public LEDs() {
        if (RobotBase.isReal()) {
            candle = new CANdle(0);
            candle.configFactoryDefault();
            CANdleConfiguration config = new CANdleConfiguration();
            config.disableWhenLOS = true;
            config.statusLedOffWhenActive = true;
            config.stripType = CANdle.LEDStripType.GRB;
            config.brightnessScalar = 1.0;
            config.v5Enabled = false;
            config.enableOptimizations = true;
            candle.configAllSettings(config);
            for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
                candle.clearAnimation(i);
            }
            candle.setLEDs(
                    0, 0, 255, 0, candleLength, stripLength); // Blue to confirm initialization
        }
    }

    public CANdle getCandle() {
        return candle;
    }
}
