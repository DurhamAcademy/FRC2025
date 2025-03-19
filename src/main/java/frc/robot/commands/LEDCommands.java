package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static frc.robot.subsystems.lights.LEDs.candleLength;
import static frc.robot.subsystems.lights.LEDs.stripLength;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lights.LEDs;

public class LEDCommands {
    public static Command flameCommand(LEDs leds, double brightness) {
        if (leds == null) return none();
        CANdle candle = leds.getCandle();
        if (candle == null) return idle(leds);
        return run(() -> {
                    // candle.animate(new RgbFadeAnimation(1.0, 0.5, candleLength, 0));
                    candle.animate(
                            new FireAnimation(
                                    brightness,
                                    0,
                                    stripLength,
                                    1,
                                    0.5,
                                    false,
                                    candleLength), // candleLength -> 0
                            0);
                    /*
                    candle.animate(
                            new FireAnimation(
                                    brightness,
                                    0,
                                    stripLength,
                                    1,
                                    0.5,
                                    true,
                                    candleLength + stripLength), //candleLength + striplength
                            2);
                    candle.animate(
                            new FireAnimation(
                                    brightness,
                                    0,
                                    stripLength,
                                    1,
                                    0.5,
                                    false,
                                    candleLength + stripLength * 2),
                            3);
                    candle.animate(
                            new FireAnimation(
                                    brightness,
                                    0,
                                    stripLength,
                                    1,
                                    0.5,
                                    true,
                                    candleLength + stripLength * 3),
                            4); **/
                })
                .finallyDo(
                        () -> {
                            for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
                                candle.clearAnimation(i);
                            }
                        })
                .ignoringDisable(true);
    }

    public static Command hasAlgae(LEDs leds, double brightness) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();

        return runEnd(
                        () -> {
                            candle.setLEDs(20, 255, 150); // sky blue
                        },
                        () -> {
                            for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
                                candle.clearAnimation(i);
                            }
                        },
                        leds)
                .ignoringDisable(true);
    }

    /**
     * TODO tell me what to do for this
     *
     * @param leds
     * @param brightness
     * @return
     */
    public static Command aligned(LEDs leds, double brightness) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();
        return startEnd(
                        () -> {
                            candle.animate(new StrobeAnimation(0, 255, 0, 0, 1, 116, 8), 0);
                            //                            candle.animate(new StrobeAnimation(0, 255,
                            // 0, 0, 1, 116, 8), 1);
                            //                            candle.animate(new StrobeAnimation(0, 255,
                            // 0, 0, 1, 116, 8), 2);
                            //                            candle.animate(new StrobeAnimation(0, 255,
                            // 0, 0, 1, 116,8), 3);
                        },
                        () -> {
                            for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
                                candle.clearAnimation(i);
                            }
                        },
                        leds)
                .ignoringDisable(true);
    }

    public static Command hasCoral(LEDs leds, double brightness) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();
        // 255, 93, 115
        // 128, 46, 58
        return runEnd(
                        () -> {
                            candle.setLEDs(255, 80, 30); // coral colour
                        },
                        () -> {
                            for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
                                candle.clearAnimation(i);
                            }
                        },
                        leds)
                .ignoringDisable(true);
    }

    public static Command blink(LEDs leds, int r, int g, int b) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();
        return run(
                        () -> {
                            boolean on = (System.currentTimeMillis() / 500) % 2 == 0;
                            if (on) {
                                candle.setLEDs(r, g, b);
                            } else {
                                candle.setLEDs(0, 0, 0);
                            }
                        },
                        leds)
                .ignoringDisable(true);
    }

    public static Command ledsUp(LEDs leds) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();

        return runEnd(
                        () -> {
                            candle.setLEDs(0, 0, 0, 0, candleLength, stripLength / 2); // up half 1
                            candle.setLEDs(
                                    100,
                                    100,
                                    100,
                                    0,
                                    candleLength + stripLength - stripLength / 2,
                                    stripLength); // up half 2 down half 2
                            candle.setLEDs(
                                    0,
                                    0,
                                    0,
                                    0,
                                    candleLength + stripLength * 2 - stripLength / 2,
                                    stripLength); // down half 1 up half 1
                            candle.setLEDs(
                                    100,
                                    100,
                                    100,
                                    0,
                                    candleLength + stripLength * 3 - stripLength / 2,
                                    stripLength); // up half 2 down half 2
                            candle.setLEDs(
                                    0,
                                    0,
                                    0,
                                    0,
                                    candleLength + stripLength * 4 - stripLength / 2,
                                    stripLength / 2); // down half 1
                        },
                        () -> {
                            for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++)
                                candle.clearAnimation(i);
                        },
                        leds)
                .ignoringDisable(true);
    }

    public static Command ledsDown(LEDs leds) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();

        return runEnd(
                () -> {
                    candle.setLEDs(100, 100, 100, 0, candleLength, stripLength / 4); // up half 1
                    candle.setLEDs(
                            0,
                            0,
                            0,
                            0,
                            candleLength + stripLength - stripLength / 2,
                            stripLength); // up half 2 down half 2
                    candle.setLEDs(
                            100,
                            100,
                            100,
                            0,
                            candleLength + stripLength * 2 - stripLength / 2,
                            stripLength); // down half 1 up half 1
                    candle.setLEDs(
                            0,
                            0,
                            0,
                            0,
                            candleLength + stripLength * 3 - stripLength / 2,
                            stripLength); // up half 2 down half 2
                    candle.setLEDs(
                            100,
                            100,
                            100,
                            0,
                            candleLength + stripLength * 4 - stripLength / 2,
                            stripLength / 2); // down half 1
                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++)
                        candle.clearAnimation(i);
                },
                leds);
    }

    // 116 individual cells
    // gamma correction cnalde class
    public static Command normal(LEDs leds) {
        if (leds == null) return idle(leds);
        if (leds.getCandle() == null) return idle(leds);

        CANdle candle = leds.getCandle();
        // can be leds.setLEDs()
        return runEnd(
                        () -> {
                            candle.setLEDs(128, 128, 128);
                        },
                        () -> {
                            for (int i = 0; candle.getMaxSimultaneousAnimationCount() > 0; i++) {
                                candle.clearAnimation(i);
                            }
                        },
                        leds)
                .ignoringDisable(true);
    }
}
