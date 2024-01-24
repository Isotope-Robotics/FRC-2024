package frc.robot.Lib.Util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorId, int angleMotorId, int cancoderID, Rotation2d angleOffset){
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
    }
}
