
package frc.lib.util.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Swerve Module IO
 */
public class SwerveModuleReal implements SwerveModuleIO {

    private CANSparkMax mAngleMotor;
    private TalonFX mDriveMotor;
    private SparkPIDController angleController;
    private CANcoder angleEncoder;
    public RelativeEncoder angleMotorEncoder;
    private TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    private CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    private StatusSignal<Double> driveMotorSelectedPosition;
    private StatusSignal<Double> driveMotorSelectedSensorVelocity;
    private StatusSignal<Double> absolutePositionAngleEncoder;
    SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());



    /** Instantiating motors and Encoders */
    public SwerveModuleReal(int moduleNumber, int driveMotorID, int angleMotorID, int cancoderID,
        Rotation2d angleOffset) {


        mDriveMotor = new TalonFX(driveMotorID);
        mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = new CANcoder(cancoderID);
        angleMotorEncoder = mAngleMotor.getEncoder();

        configAngleEncoder();
        configAngleMotor();
        configDriveMotor();

        driveMotorSelectedPosition = mDriveMotor.getPosition();
        driveMotorSelectedSensorVelocity = mDriveMotor.getVelocity();
        absolutePositionAngleEncoder = angleEncoder.getAbsolutePosition();
        mAngleMotor.restoreFactoryDefaults();



    }

    private void configAngleMotor() {
        /* Angle Motor Config */

        /* Motor Inverts and Neutral Mode */
        this.mAngleMotor.setInverted(false);
        this.mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);

        /* Gear Ratio and Wrapping Config */
        // swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        // swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        // /* Current Limiting */
        this.mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleCurrentLimit);
        this.mAngleMotor.setSecondaryCurrentLimit(Constants.Swerve.angleCurrentThreshold);
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold =
        // Constants.Swerve.angleCurrentThreshold;
        // swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold =
        // Constants.Swerve.angleCurrentThresholdTime;

        // /* PID Config */
        this.angleController = mAngleMotor.getPIDController();
        this.angleController.setFeedbackDevice(this.angleMotorEncoder);
        this.angleController.setP(Constants.Swerve.angleKP);
        this.angleController.setI(Constants.Swerve.angleKI);
        this.angleController.setD(Constants.Swerve.angleKD);
        this.angleController.setOutputRange(Constants.Swerve.angleMinOutput,
            Constants.Swerve.angleMaxOutput);

        this.angleMotorEncoder.setPositionConversionFactor(Constants.Swerve.angleGearRatio);
        this.angleMotorEncoder.setVelocityConversionFactor(Constants.Swerve.angleGearRatio);
        this.angleController.setPositionPIDWrappingEnabled(true);
        this.angleController.setPositionPIDWrappingMinInput(-0.5);
        this.angleController.setPositionPIDWrappingMaxInput(0.5);

        this.mAngleMotor.burnFlash();
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
    public void setAngleMotor(double v) {
        angleController.setReference(v, CANSparkBase.ControlType.kPosition);
    }


    @Override
    public void setDriveMotor(ControlRequest request) {
        mDriveMotor.setControl(request);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        BaseStatusSignal.refreshAll(driveMotorSelectedPosition, driveMotorSelectedSensorVelocity,
            absolutePositionAngleEncoder);
        inputs.driveMotorSelectedPosition = driveMotorSelectedPosition.getValue();
        inputs.driveMotorSelectedSensorVelocity = driveMotorSelectedSensorVelocity.getValue();
        inputs.angleMotorSelectedPosition = angleMotorEncoder.getPosition();
        inputs.absolutePositionAngleEncoder = absolutePositionAngleEncoder.getValue();
    }


    @Override
    public void setPositionAngleMotor(double absolutePosition) {
        angleMotorEncoder.setPosition(absolutePosition);
    }

}
