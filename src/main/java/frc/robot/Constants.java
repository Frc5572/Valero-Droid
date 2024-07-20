package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * Constants file.
 */
public final class Constants {
    /**
     * Stick Deadband
     */
    public static final double STICK_DEADBAND = 0.1;
    /**
     * Driver ID
     */
    public static final int driverID = 0;
    /**
     * Operator ID
     */
    public static final int operatorID = 1;

    /**
     * Motor CAN id's.
     */
    public static final class Motors {
    }


    /**
     * Swerve Constants
     */
    public static final class Swerve {
        public static final double AUTO_ROTATION_KP = 5.0;
        public static final double AUTO_ROTATION_KI = 0.0;
        public static final double AUTO_ROTATION_KD = 0.0;

        public static final edu.wpi.first.wpilibj.SPI.Port navXID =
            edu.wpi.first.wpilibj.SPI.Port.kMXP;
        public static final boolean invertGyro = true;
        public static final boolean isFieldRelative = true;
        public static final boolean isOpenLoop = false;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.75);
        public static final double wheelBase = Units.inchesToMeters(17.75);
        public static final double wheelDiameter = Units.inchesToMeters(3.8);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        public static final Translation2d MOD0_MODOFFSET =
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0);

        /*
         * Swerve Kinematics No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = (8.14 / 1.0); // MK4i L1
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // (150 / 7) : 1

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue driveMotorInvert =
            InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 100.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 10.0;
        public static final double AUTO_MAX_SPEED = 3.0;
        /** Radians per Second */
        public static final double maxAngularVelocity = 15.0;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */

        /**
         * Front Left Module - Module 0
         */
        public static final class Mod0 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 51;
            public static final int canCoderID = 4;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(183.955078125);
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.004395 + 0.5);

        }

        /**
         * Front Right Module - Module 1
         */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 40;
            public static final int canCoderID = 2;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(325.01953125);
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.415527 + 0.5);

        }

        /**
         * Back Left Module - Module 2
         */
        public static final class Mod2 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 1;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(124.62890625);
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.145264 + 0.5);

        }

        /**
         * Back Right Module - Module 3
         */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 10;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(295.400390625);
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.323730 + 0.5);
        }

        public static final HolonomicPathFollowerConfig pathFollowerConfig =
            new HolonomicPathFollowerConfig(new PIDConstants(5.0, 0, 0),
                new PIDConstants(AUTO_ROTATION_KP, AUTO_ROTATION_KI, AUTO_ROTATION_KD),
                // Drive base radius (distance from center to furthest module)
                AUTO_MAX_SPEED, MOD0_MODOFFSET.getNorm(), new ReplanningConfig());
    }

    /**
     * Climber constants
     */
    public static final class ClimberConstants {
        public static final double CLIMBER_KP = 0.1;
        public static final double CLIMBER_KI = 0.1;
        public static final double CLIMBER_KD = 0.1;
        public static final double CLIMBER_MAX_VELOCITY = 0;
        public static final double CLIMBER_MAX_ACCELERATION = 0;
        public static final double CLIMBER_KS = 0.1;
        public static final double CLIMBER_KG = 0.1;
        public static final double CLIMBER_KV = 0.1;
        public static final double CLIMBER_POWER = 0.8;

        public static final double CLIMBING_DISTANCE = Units.inchesToMeters(15);
        public static final double MAX_CLIMBING_DISTANCE = Units.inchesToMeters(21);

        // 2pi * radius
        public static final double LINEAR_DISTANCE = Units.inchesToMeters(2 * Math.PI * 1);
    }

    /**
     * Auto constants
     */
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }

}
