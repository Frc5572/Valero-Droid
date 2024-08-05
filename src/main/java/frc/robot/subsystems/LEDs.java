package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/**
 * This is the class header for the LEDs Subsystem
 */
public class LEDs extends SubsystemBase {

    private int start;
    private int length;
    private int end;

    /**
     *
     * @param start
     * @param length
     */
    public LEDs(int start, int length) {
        this.start = start;
        this.length = length;
        this.end = start + length;
    }

    /**
     * Set individual LED color via HSV
     *
     * @param index the LED index to set
     * @param h the h value [0-180)
     * @param s the s value [0-255]
     * @param v the v value [0-255]
     */
    public void setHSV(int index, int h, int s, int v) {
        Robot.controLedBuffer.setHSV(index, h, s, v);
    }

    /**
     * Set individual LED color via RGB
     *
     * @param index the LED index to set
     * @param r the r value [0-255]
     * @param g the g value [0-255]
     * @param b the b value [0-255]
     */
    public void setRGB(int index, int r, int g, int b) {
        Robot.controLedBuffer.setRGB(index, r, g, b);
    }

    public int getEnd() {
        return end;
    }

    public int getStart() {
        return start;
    }

    public int getLength() {
        return length;
    }

    /**
     * Sets RGB Color of the entire LED strip
     *
     * @param r - [0 - 255]
     * @param g - [0 - 255]
     * @param b - [0 - 255]
     */
    public void setRGB(int r, int g, int b) {
        for (var i = getStart(); i < getEnd(); i++) {
            setRGB(i, r, g, b);
        }
    }

    /**
     * Set individual LED color via Color
     *
     * @param index the LED index to set
     * @param color The color of the LED
     */
    public void setColor(int index, Color color) {
        Robot.controLedBuffer.setLED(index, color);
    }

    /**
     * Sets the Color of the entire LED strip
     *
     * @param color color set for the LEDs
     */
    public void setColor(Color color) {
        for (var i = getStart(); i < getEnd(); i++) {
            setColor(i, color);
        }
    }

    public Color getColor(int index) {
        return Robot.controLedBuffer.getLED(index);
    }

    /**
     * Set Static Color
     *
     * @param color color of LED strip
     * @return Command
     */
    public Command setStaticColor(Color color) {
        return Commands.run(() -> setColor(color), this).ignoringDisable(true);
    }
}
