package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

/**
 * Real Turret
 */
public class TurretReal implements TurretIO {

    private SparkFlex turretMotor =
        new SparkFlex(Constants.TurretConstants.motorID, MotorType.kBrushless);
    private RelativeEncoder relativeEncoder;
    private DutyCycleEncoder absoluteEncoder =
        new DutyCycleEncoder(Constants.TurretConstants.encoderPort);

    /** Real Swerve Initializer */
    public TurretReal() {
        relativeEncoder = turretMotor.getEncoder();
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.absoluteEncoderPos = absoluteEncoder.get();
        inputs.motorVelocity = relativeEncoder.getVelocity();
        inputs.relativeEncoderPos = relativeEncoder.getPosition();
    }

    public void turnTurret(double power) {
        turretMotor.set(power);
    }
}
