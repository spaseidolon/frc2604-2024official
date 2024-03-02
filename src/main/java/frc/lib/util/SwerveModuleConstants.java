package frc.lib.util;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int encoderID;
    public final double angleOffset;
    public final boolean forceAbsolute;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param encoderID
     * @param angleOffset
     * @param forceAbsolute
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int encoderID, double angleOffset, boolean forceAbsolute) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.encoderID = encoderID;
        this.angleOffset = angleOffset;
        this.forceAbsolute = forceAbsolute;
    }
}
