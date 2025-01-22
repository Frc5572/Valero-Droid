package frc.lib.util.swerve;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** IO Class for SwerveModule */
public interface SwerveModuleIO {
    /** Inputs Class for SwerveModule */
    @AutoLog
    public static class SwerveModuleInputs {
        public Angle driveMotorSelectedPosition;
        public AngularVelocity driveMotorSelectedSensorVelocity;
        public Angle angleMotorSelectedPosition;
        public Angle absolutePositionAngleEncoder;

    }

    public default void updateInputs(SwerveModuleInputs inputs) {}

    public default void setDriveMotor(ControlRequest request) {}

    public default void setAngleMotor(double v) {}


    public default void setAngleSelectedSensorPosition(double angle) {}

    public default void setPositionAngleMotor(double absolutePosition) {}



}
