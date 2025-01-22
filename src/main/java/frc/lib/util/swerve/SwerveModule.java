
package frc.lib.util.swerve;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.robot.Constants;

/**
 * Swerve Module Subsystem
 */
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    // private double lastAngle;

    private SwerveModuleIO io;
    private SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
        Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /**
     * Swerve Module
     *
     * @param moduleNumber Module Number
     * @param driveMotorID CAN ID of the Drive Motor
     * @param angleMotorID CAN ID of the Angle Motor
     * @param cancoderID CAN ID of the CANCoder
     * @param angleOffset Angle Offset of the CANCoder to align the wheels
     */
    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int cancoderID,
        Rotation2d angleOffset, SwerveModuleIO io) {
        this.io = io;

        this.moduleNumber = moduleNumber;

        this.angleOffset = angleOffset;

        // lastAngle = getState().angle.getDegrees();
        io.updateInputs(inputs);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        resetToAbsolute();
        Logger.processInputs("SwerveModule" + moduleNumber, inputs);
    }

    /**
     * Periodic function
     */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SwerveModule" + moduleNumber, inputs);

    }

    /**
     * Set the desired state of the Swerve Module
     *
     * @param desiredState The desired {@link SwerveModuleState} for the module
     * @param isOpenLoop Whether the state should be open or closed loop controlled
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.optimize(getState().angle);
        io.setAngleMotor(desiredState.angle.getRotations());
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Set the velocity or power of the drive motor
     *
     * @param desiredState The desired {@link SwerveModuleState} of the module
     * @param isOpenLoop Whether the state should be open or closed loop controlled
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            io.setDriveMotor(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.metersPerSecondToRotationPerSecond(
                desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward =
                driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            io.setDriveMotor(driveVelocity);
        }
    }

    /**
     * Get the rotation of the CANCoder
     *
     * @return The rotation of the CANCoder in {@link Rotation2d}
     */
    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(inputs.absolutePositionAngleEncoder.in(Rotations));
    }

    /**
     * Reset the Swerve Module angle to face forward
     */
    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        io.setPositionAngleMotor(absolutePosition);
        inputs.angleMotorSelectedPosition = Rotations.of(absolutePosition);
    }

    /**
     * Get the current Swerve Module State
     *
     * @return The current {@link SwerveModuleState}
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.rotationPerSecondToMetersPerSecond(
                inputs.driveMotorSelectedSensorVelocity.in(RotationsPerSecond),
                Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(inputs.angleMotorSelectedPosition.in(Rotations)));
    }

    // /**
    // * Get the absolute current Swerve Module State
    // *
    // * @return The current {@link SwerveModuleState}
    // */
    // public SwerveModuleState getAbsoluteState() {
    // return new SwerveModuleState(
    // Conversions.rotationPerSecondToMetersPerSecond(
    // inputs.driveMotorSelectedSensorVelocity.in(RotationsPerSecond),
    // Constants.Swerve.wheelCircumference),
    // Rotation2d.fromRotations(inputs.absolutePositionAngleEncoder.in(Rotations)));
    // }


    /**
     * Get the current Swerve Module Position
     *
     * @return The current {@link SwerveModulePosition}
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(inputs.driveMotorSelectedPosition.in(Rotations),
                Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(inputs.angleMotorSelectedPosition.in(Rotations)));
    }

}
