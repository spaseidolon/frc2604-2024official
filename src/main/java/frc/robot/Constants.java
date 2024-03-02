package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 20;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(17.5);
        public static final double wheelBase = Units.inchesToMeters(25.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.7);//change to 3.7ish for MK4s when sure 
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (8.14 / 1.0); //6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 3;
        public static final double angleKI = 0;
        public static final double angleKD = 0;
        public static final double angleKF = 0.9;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.5;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Right Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int encoderID = 0;
            public static final double angleOffset = 222.4;
            public static final boolean forceabsolute = true;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, forceabsolute);
        }

        /* Front Left Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int encoderID = 2;
            public static final double angleOffset = 200.4;
            public static final boolean forceabsolute = true;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, forceabsolute);
        }



        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int encoderID = 3;
            public static final double angleOffset = 334.5;
            public static final boolean forceabsolute = true;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, forceabsolute);
        }


        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int encoderID = 1;
            public static final double angleOffset = 61.6;
            public static final boolean forceabsolute = true;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, forceabsolute);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 2;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
