package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    public static final int stripLength = 16;
    public static final int candleLength = 8;

    public CANdle candle;

    public LEDs() {
        if(RobotBase.isReal()) {
            candle = new CANdle(0);
            candle.configFactoryDefault(); // puts in default settings idrk
            CANdleConfiguration config = new CANdleConfiguration(); // more built in configurations
            config.disableWhenLOS = true;
            config.statusLedOffWhenActive = true;
            config.stripType = CANdle.LEDStripType.GRB;
            config.brightnessScalar = 1.0;
            config.v5Enabled = false;
            config.enableOptimizations = true;
            candle.configAllSettings(config);
            for(int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
                candle.clearAnimation(i);
            }
        }




    }
    public void periodic() {
        // help i dont know what to put here
    }
    public CANdle getCandle() { return candle; }
    public void setLEDs(int r, int g, int b) {candle.setLEDs(r, g, b); }
    public void resetLEDs() { candle.setLEDs(0,0,0); }


}

