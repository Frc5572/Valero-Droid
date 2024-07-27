package frc.robot.subsystems.lightsabers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lightsaber extends SubsystemBase {

    private LightsaberIO io;
    private LightsaberInputsAutoLogged inputs = new LightsaberInputsAutoLogged();

    public Lightsaber(LightsaberIO lightsaberIO) {
        this.io = lightsaberIO;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
    }

    public Command turnLightsabers(double power) {
        return Commands.startEnd(() -> io.setMotor(power), () -> io.setMotor(0), this);
    }
}
