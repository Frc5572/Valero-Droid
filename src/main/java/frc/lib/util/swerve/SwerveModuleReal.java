
package frc.lib.util.swerve;

import java.util.function.Supplier;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/**
 * Swerve Module IO
 */
public class SwerveModuleReal implements SwerveModuleIO {

    private CANSparkMax mAngleMotor;
    private TalonFX mDriveMotor;
    private PIDController angleController;
    private AbsoluteEncoder angleMotorEncoder;
    private CANcoder angleEncoder;
    private TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    private CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    private StatusSignal<Double> driveMotorSelectedPosition;
    private StatusSignal<Double> driveMotorSelectedSensorVelocity;
    private Supplier<Double> angleMotorSelectedPosition;
    private StatusSignal<Double> absolutePositionAngleEncoder;
    private double pidValue = 0;


    /** Instantiating motors and Encoders */
    public SwerveModuleReal(int moduleNumber, int driveMotorID, int angleMotorID, int cancoderID,
        Rotation2d angleOffset) {


        mDriveMotor = new TalonFX(driveMotorID);
        mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = new CANcoder(cancoderID);
        angleMotorEncoder =
            mAngleMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);

        configAngleEncoder();
        configAngleMotor();
        configDriveMotor();

        angleEncoder.getPosition();
        driveMotorSelectedPosition = mDriveMotor.getPosition();
        driveMotorSelectedSensorVelocity = mDriveMotor.getVelocity();
        angleMotorSelectedPosition = () -> angleMotorEncoder.getPosition();
        absolutePositionAngleEncoder = angleEncoder.getAbsolutePosition();
        pidValue = absolutePositionAngleEncoder.getValue();

    }

    private void configAngleMotor() {
        /* Angle Motor Config */
        this.mAngleMotor.burnFlash();

        /* Motor Inverts and Neutral Mode */
        this.mAngleMotor.setInverted(false);
        this.mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);

        /* Gear Ratio and Wrapping Config */
        // swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        // swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        // /* Current Limiting */
        this.mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleCurrentLimit);
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold =
        // Constants.Swerve.angleCurrentThreshold;
        // swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold =
        // Constants.Swerve.angleCurrentThresholdTime;

        // /* PID Config */
        angleController = new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI,
            Constants.Swerve.angleKD);
        this.angleMotorEncoder.setPositionConversionFactor(Constants.Swerve.angleEncoderRot2Rad);
        this.angleMotorEncoder
            .setVelocityConversionFactor(Constants.Swerve.angleEncoderRPM2RadPerSec);
        // mAngleMotor.getConfigurator().apply(swerveAngleFXConfig);
    }

    private void configDriveMotor() {
        /* Drive Motor Config */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;


        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
            Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold =
            Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold =
            Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
            Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
            Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
            Constants.Swerve.closedLoopRamp;

        mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    private void configAngleEncoder() {
        /* Angle Encoder Config */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        angleEncoder.getConfigurator().apply(swerveCANcoderConfig);
    }



    @Override
    public void setAngleMotor(double request) {
        mAngleMotor.set(angleController.calculate(request, pidValue));
    }

    @Override
    public void setDriveMotor(ControlRequest request) {
        mDriveMotor.setControl(request);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        BaseStatusSignal.refreshAll(driveMotorSelectedPosition, driveMotorSelectedSensorVelocity);
        inputs.driveMotorSelectedPosition = driveMotorSelectedPosition.getValue();
        inputs.driveMotorSelectedSensorVelocity = driveMotorSelectedSensorVelocity.getValue();
        inputs.angleMotorSelectedPosition = angleMotorSelectedPosition.get();
        inputs.absolutePositionAngleEncoder = absolutePositionAngleEncoder.getValue();
        // inputs.driveMotorTemp = mDriveMotor.getDeviceTemp().getValueAsDouble();
        // inputs.angleMotorTemp = mAngleMotor.getDeviceTemp().getValueAsDouble();
    }

}
