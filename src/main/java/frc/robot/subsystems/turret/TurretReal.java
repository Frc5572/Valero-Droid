package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

/**
 * Real Turret
 */
public class TurretReal implements TurretIO {

    private CANSparkFlex turretMotor =
        new CANSparkFlex(Constants.TurretConstants.motorID, MotorType.kBrushless);
    private RelativeEncoder relativeEncoder;
    private DutyCycleEncoder absoluteEncoder =
        new DutyCycleEncoder(Constants.TurretConstants.encoderPort);

    /** Real Swerve Initializer */
    public TurretReal() {
        relativeEncoder = turretMotor.getEncoder();
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.absoluteEncoderPos = absoluteEncoder.getAbsolutePosition();
        inputs.motorVelocity = relativeEncoder.getVelocity();
        inputs.relativeEncoderPos = relativeEncoder.getPosition();
    }

    public void turnTurret(double power) {
        turretMotor.set(power);
    }
}
