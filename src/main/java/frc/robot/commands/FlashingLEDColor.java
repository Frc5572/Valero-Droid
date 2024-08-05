package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

/** Command to flash LEDs on and off once per second. */
public class FlashingLEDColor extends Command {
    private LEDs leds;
    private int ledLength;
    private int flashingDelay = 0;
    private Color color;
    private Color altColor;
    private int flashDelay = 20;
    private boolean pulse = true;

    /**
     * Command to flash the LED strip between 2 colors
     *
     * @param leds LED Subsystem
     * @param color The first color
     * @param altColor The second color
     */
    public FlashingLEDColor(LEDs leds, Color color, Color altColor) {
        this.leds = leds;
        this.color = color;
        this.altColor = altColor;
        ledLength = leds.getLength();
        addRequirements(leds);
    }

    /**
     * Command to flash the LED strip between 2 colors
     *
     * @param leds LED Subsystem
     * @param color The first color
     * @param altColor The second color
     */
    public FlashingLEDColor(LEDs leds, Color color, Color altColor, int delay) {
        this(leds, color, altColor);
        flashDelay = delay;
    }

    /**
     * Command to flash the LED strip between a color and black
     *
     * @param leds LED Subsystem
     * @param color The color
     */
    public FlashingLEDColor(LEDs leds, Color color) {
        this(leds, color, Color.kBlack);
    }

    @Override
    public void execute() {
        double value = Math.abs(Math.sin(flashingDelay * Math.PI / flashDelay) * 255);
        if (flashingDelay < flashDelay) {
            if (pulse) {
                int red = (int) (color.red * value);
                int green = (int) (color.green * value);
                int blue = (int) (color.blue * value);
                leds.setRGB(red, green, blue);
            } else {
                leds.setColor(color);
            }
        } else {
            if (pulse) {
                int red = (int) (altColor.red * value);
                int green = (int) (altColor.green * value);
                int blue = (int) (altColor.blue * value);
                leds.setRGB(red, green, blue);
            } else {
                leds.setColor(altColor);
            }
        }
        flashingDelay++;
        flashingDelay %= flashDelay * 2;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
