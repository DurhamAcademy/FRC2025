package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// program it like a motor
// because blinkin doesn't have built in documentation

public class LEDs extends SubsystemBase {
    private final PWMSparkMax blinkin;

    public LEDs(int pwmPort) {
        blinkin = new PWMSparkMax(pwmPort);
    }

    public void setLED(double value) {
        blinkin.set(value); // Set PWM value based on Blinkin chart
    }

    public void defaultRainbow() {
        blinkin.set(-0.97); // does colorful things with rainbow
    }

    public void solidColor(String colorName) {
        switch (colorName) {
            case "red":
                blinkin.set(0.61);
                break;
            case "orange":
                blinkin.set(0.65);
                break;
            case "yellow":
                blinkin.set(0.69);
                break;
            case "green":
                blinkin.set(0.77);
                break;
            case "blue":
                blinkin.set(0.87);
                break;
            case "violet":
                blinkin.set(0.91);
                break;
            default:
                blinkin.set(0.63);
        }
    }
}
