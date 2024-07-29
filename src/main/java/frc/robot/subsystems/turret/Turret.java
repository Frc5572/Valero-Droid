package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    @AutoLogOutput(key = "Turret/TurretPosition")
    public double getTurretPosition() {
        double pos =
            inputs.absoluteEncoderPos + (1.0 - Constants.TurretConstants.absoluteEncoderOffset);
        return pos > 1.0 ? pos - 1.0 : pos;
    }

    public Command turnTurret(double power) {
        return Commands.startEnd(() -> turretIO.turnTurret(power), () -> turretIO.turnTurret(0),
            this);
    }
}
