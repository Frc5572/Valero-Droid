package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    /** Inputs Class for Turret */

    @AutoLog
    public static class TurretInputs {
        public double absoluteEncoderPos;
        public double relativeEncoderPos;
        public double motorVelocity;
    }

    public default void updateInputs(TurretInputs inputs) {}

    public default void turnTurret(double power) {}

}
