package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Turret Subsystem
 */
public class Turret extends SubsystemBase {

    private TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();
    private TurretIO turretIO;
    // private PIDController pidController = new PIDController(0, 0, 0);

    public Turret(TurretIO io) {
        this.turretIO = io;
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    /**
     * Get Turret Absolute position
     *
     * @return The absolute position
     */
    @AutoLogOutput(key = "Turret/TurretPosition")
    public double getTurretPosition() {
        double pos =
            inputs.absoluteEncoderPos + (1.0 - Constants.TurretConstants.absoluteEncoderOffset);
        return pos > 1.0 ? pos - 1.0 : pos;
    }

    /**
     * Command to turn the turrent
     *
     * @param power Power between -1 to 1
     * @return Command
     */
    public Command turnTurret(double power) {
        return Commands.startEnd(() -> turretIO.turnTurret(power), () -> turretIO.turnTurret(0),
            this);
    }

    /**
     * Command to turn the turrent clockwise
     *
     * @param power Power between -1 to 1
     * @return Command
     */
    public Command turnTurretClockwise(double power) {
        return this.turnTurret(power).unless(() -> this.getTurretPosition() > .95)
            .until(() -> this.getTurretPosition() > .95);
    }

    /**
     * Command to turn the turrent counter-clockwise
     *
     * @param power Power between -1 to 1
     * @return Command
     */
    public Command turnTurretCounterClockwise(double power) {
        return this.turnTurret(-power).unless(() -> this.getTurretPosition() < .05)
            .until(() -> this.getTurretPosition() < .05);
    }

    /**
     * Command to turn the back and forth
     *
     * @param power Power between -1 to 1
     * @return Command
     */
    public Command turnBackandForth(double power) {
        Command conditional = Commands.either(this.turnTurretClockwise(power),
            this.turnTurretCounterClockwise(power), () -> this.getTurretPosition() < 0.5);
        return Commands.repeatingSequence(conditional);
    }
}
