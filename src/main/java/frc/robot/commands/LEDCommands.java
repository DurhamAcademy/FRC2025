package frc.robot.commands;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotController;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.lights.LEDs;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static frc.robot.subsystems.lights.LEDs.candleLength;
import static frc.robot.subsystems.lights.LEDs.stripLength;

public class LEDCommands {
    public static Command flameCommand(LEDs leds, double brightness) {
        if(leds == null) return none();
        if(leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();
        return startEnd(
                () -> {
                    candle.animate(new RgbFadeAnimation(1.0, 0.5, candleLength, 0));
                    candle.animate(new FireAnimation(brightness, 0, stripLength, 1, 0.5, false, candleLength), 1);
                    candle.animate(new FireAnimation(brightness, 0, stripLength, 1, 0.5, true, candleLength + stripLength), 2);
                    candle.animate(new FireAnimation(brightness, 0, stripLength, 1, 0.5, false, candleLength + stripLength*2), 3);
                    candle.animate(new FireAnimation(brightness, 0, stripLength, 1, 0.5, true, candleLength + stripLength*3), 4);
                },
                () -> {
                    for(int i =0; i <candle.getMaxSimultaneousAnimationCount(); i++){
                        candle.clearAnimation(i);
                    }
                },
                leds

        );
    }

    public static Command hasAlgae(LEDs leds, double brightness) {
        if(leds == null) return none();
        if(leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();

        return startEnd(
                () -> {
                    candle.animate(new LarsonAnimation(255, 165, 0, 0, 0, stripLength, LarsonAnimation.BounceMode.Back, stripLength, candleLength), 0);
                    candle.animate(new LarsonAnimation(255, 165, 0, 0, 0, stripLength, LarsonAnimation.BounceMode.Front, stripLength, candleLength + stripLength), 1);
                    candle.animate(new LarsonAnimation(255, 165, 0, 0, 0, stripLength, LarsonAnimation.BounceMode.Back, stripLength, candleLength + stripLength*2), 2);
                    candle.animate(new LarsonAnimation(255, 165, 0, 0, 0, stripLength, LarsonAnimation.BounceMode.Front, stripLength, candleLength + stripLength*3), 3);
                },
                () -> {
                    for(int i =0; i <candle.getMaxSimultaneousAnimationCount(); i++){
                        candle.clearAnimation(i);
                    }
                },
                leds
        );

    }

    /**
     * TODO tell me what to do for this
     * @param leds
     * @param brightness
     * @return
     */
    public static Command aligned(LEDs leds, double brightness) {return null; }
    public static Command hasCoral(LEDs leds, double brightness) {return null; }
    public static Command ryanLandisBaurothTheUltimateKingHeIsMyEternalGoat(LEDs leds, double brightness) {
        if(leds == null) return none();
        if(leds.getCandle() == null) return idle(leds);
        CANdle candle = leds.getCandle();

        return startEnd(
                () -> {
                    candle.animate(new RainbowAnimation(brightness, 0, 0, true, 0));
                    candle.animate(new RainbowAnimation(brightness, 0, 0, false, 1));
                    candle.animate(new RainbowAnimation(brightness, 0, 0, true, 2));
                    candle.animate(new RainbowAnimation(brightness, 0, 0, false, 3));
                },
                () -> {
                    for(int i =0; i <candle.getMaxSimultaneousAnimationCount(); i++){
                        candle.clearAnimation(i);
                    }
                },
                leds

        );
    }
}
