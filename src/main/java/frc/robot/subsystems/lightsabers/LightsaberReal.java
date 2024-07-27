package frc.robot.subsystems.lightsabers;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

/**
 * Lightsaber Real
 */
public class LightsaberReal implements LightsaberIO {

    private CANSparkFlex lighstaberMotor1;
    private CANSparkFlex lighstaberMotor2;
    private CANSparkFlex lighstaberMotor3;

    public LightsaberReal() {
        lighstaberMotor1 =
            new CANSparkFlex(Constants.LightsaberConstants.ls1, MotorType.kBrushless);
        lighstaberMotor2 =
            new CANSparkFlex(Constants.LightsaberConstants.ls2, MotorType.kBrushless);
        lighstaberMotor3 =
            new CANSparkFlex(Constants.LightsaberConstants.ls3, MotorType.kBrushless);

        lighstaberMotor1.restoreFactoryDefaults();
        lighstaberMotor2.restoreFactoryDefaults();
        lighstaberMotor3.restoreFactoryDefaults();
        lighstaberMotor1.setInverted(Constants.LightsaberConstants.inverted);
        lighstaberMotor2.setInverted(Constants.LightsaberConstants.inverted);
        lighstaberMotor3.setInverted(Constants.LightsaberConstants.inverted);
        lighstaberMotor2.setIdleMode(Constants.LightsaberConstants.brakeMode);
        lighstaberMotor1.setIdleMode(Constants.LightsaberConstants.brakeMode);
        lighstaberMotor3.setIdleMode(Constants.LightsaberConstants.brakeMode);
        lighstaberMotor1.burnFlash();
        lighstaberMotor2.burnFlash();
        lighstaberMotor3.burnFlash();
    }

    @Override
    public void setMotor(double power) {
        lighstaberMotor1.set(power);
        lighstaberMotor2.set(power);
        lighstaberMotor3.set(power);
    }
}
