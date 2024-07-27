package frc.robot.subsystems.lightsabers;

import org.littletonrobotics.junction.AutoLog;

public interface LightsaberIO {
    /** Inputs Class for Swerve */

    @AutoLog
    public static class LightsaberInputs {
        public double ls1Velocity;
        public double ls2Velocity;
        public double ls3Velocity;
    }

    public default void updateInputs(LightsaberInputs inputs) {}

    public default void setMotor(double power) {}
}
