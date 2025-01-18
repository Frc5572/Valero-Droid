package frc.robot.subsystems.lightsabers;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * Lightsaber Subsystem
 */
public class Lightsaber extends SubsystemBase {

    private LightsaberIO io;
    private LightsaberInputsAutoLogged inputs = new LightsaberInputsAutoLogged();

    /**
     * Lighstaber Constructor
     *
     * @param lightsaberIO IO Class
     */
    public Lightsaber(LightsaberIO lightsaberIO) {
        this.io = lightsaberIO;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Lightsabers", inputs);
    }

    /**
     * Command to spin the lightsabers
     *
     * @param power Power to supply to the motors
     * @return Command
     */
    public Command turnLightsabers(double power) {
        return Commands.startEnd(() -> io.setMotor(power), () -> io.setMotor(0), this);
    }
}
