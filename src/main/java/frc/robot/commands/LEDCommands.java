package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static frc.robot.subsystems.lights.LEDs.candleLength;
import static frc.robot.subsystems.lights.LEDs.stripLength;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lights.LEDs;
import java.util.function.BooleanSupplier;

public class LEDCommands {

    public static Command aligned(LEDs leds) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();
        return startEnd(
                () -> {
                    candle.animate(new StrobeAnimation(0, 255, 0, 0, 0.2, stripLength, candleLength));
                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
                        candle.clearAnimation(i);
                    }
                },
                leds);
    }

    public static Command blink(LEDs leds, int r, int g, int b) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();
        return startEnd(
                () -> {
                    candle.animate(new StrobeAnimation(r, g, b, 0, 0.5, stripLength, candleLength));
                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
                        candle.clearAnimation(i);
                    }
                },
                leds);
    }

    public static Command enabled(LEDs leds, BooleanSupplier algaeMode) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();
        return runEnd(
                () -> {
                    if (algaeMode.getAsBoolean()) {
                        candle.setLEDs(0, 128, 128, 0, 0, stripLength);
                    } else {
                        candle.setLEDs(248, 131, 121, 0, 0, stripLength);
                    }
                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++)
                        candle.clearAnimation(i);
                },
                leds);
    }

    public static Command disabled(LEDs leds) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();

        return startEnd(
                () ->
                        candle.animate(
                                new LarsonAnimation(
                                        248,
                                        131,
                                        121,
                                        0,
                                        0.1,
                                        stripLength,
                                        LarsonAnimation.BounceMode.Center,
                                        10,
                                        candleLength)),
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++)
                        candle.clearAnimation(i);
                },
                leds);
    }

    public static Command flameCommand(LEDs leds) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        return flameCommand(leds, 0.5);
    }

    public static Command flameCommand(LEDs leds, double brightness) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();
        return startEnd(
                () -> {
                    candle.animate(new RgbFadeAnimation(1.0, 0.5, candleLength, 0));
                    candle.animate(
                            new FireAnimation(
                                    brightness, 0.5, stripLength, 1, .5, false, candleLength));
                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++)
                        candle.clearAnimation(i);
                },
                leds);
    }
}
