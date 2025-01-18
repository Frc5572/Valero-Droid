package frc.robot.subsystems.lightsabers;

//import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

/**
 * Lightsaber Real
 */
public class LightsaberReal implements LightsaberIO {
    private SparkFlex lighstaberMotor1;
    private SparkFlex lighstaberMotor2;
    private SparkFlex lighstaberMotor3;
    private SparkBaseConfig config;

    /**
     * Lightsaber Constructor
     */
    public LightsaberReal() {
        lighstaberMotor1 =
            new SparkFlex(Constants.LightsaberConstants.ls1, MotorType.kBrushless);
        lighstaberMotor2 =
            new SparkFlex(Constants.LightsaberConstants.ls2, MotorType.kBrushless);
        lighstaberMotor3 =
            new SparkFlex(Constants.LightsaberConstants.ls3, MotorType.kBrushless);
        config = new SparkMaxConfig();

        lighstaberMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lighstaberMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lighstaberMotor3.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //lighstaberMotor1.restoreFactoryDefaults();
        //lighstaberMotor2.restoreFactoryDefaults();
        //lighstaberMotor3.restoreFactoryDefaults();

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        //lighstaberMotor1.setInverted(Constants.LightsaberConstants.inverted);
        //lighstaberMotor2.setInverted(Constants.LightsaberConstants.inverted);
        //lighstaberMotor3.setInverted(Constants.LightsaberConstants.inverted);
        lighstaberMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lighstaberMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lighstaberMotor3.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //lighstaberMotor2.setIdleMode(Constants.LightsaberConstants.brakeMode);
        //lighstaberMotor1.setIdleMode(Constants.LightsaberConstants.brakeMode);
        //lighstaberMotor3.setIdleMode(Constants.LightsaberConstants.brakeMode);
        //lighstaberMotor1.burnFlash();
        //lighstaberMotor2.burnFlash();
        //lighstaberMotor3.burnFlash();
    }

    @Override
    public void setMotor(double power) {
        lighstaberMotor1.set(power);
        lighstaberMotor2.set(power);
        lighstaberMotor3.set(power);
    }
}
