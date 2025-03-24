package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static frc.robot.subsystems.lights.LEDs.candleLength;
import static frc.robot.subsystems.lights.LEDs.stripLength;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.lights.LEDs;

import java.util.Optional;
import java.util.function.BooleanSupplier;

public class LEDCommands {

    public static Command aligned(LEDs leds) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();
        return startEnd(
                        () -> {
                            candle.animate(new StrobeAnimation(0, 255, 0, 0, 0, stripLength, 0), 0);
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
                    candle.animate(new StrobeAnimation(r, g, b, 0, 0, stripLength, 0), 0);
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
        var candle = leds.getCandle();
        return startEnd(
                () -> {
                    Commands.either(
                            Commands.runOnce(() -> candle.setLEDs(0, 128, 128)),
                            Commands.runOnce(() -> candle.setLEDs(248, 131, 121)),
                            algaeMode);
                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) candle.clearAnimation(i);
                },
                leds
        );
    }

    public static Command disabled(LEDs leds) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();

        return startEnd(
                () -> Commands.runOnce(() -> candle.setLEDs(248, 131, 121)),
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) candle.clearAnimation(i);
                },
                leds
        );
        }
}
