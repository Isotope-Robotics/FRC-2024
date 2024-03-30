package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Lib.Math.Conversions;
import frc.robot.Lib.Util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mDriveMotor;
    private TalonFX mAngleMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS,
            Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // Angle Encoder Config
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        // Angle Motor Config
        mAngleMotor = new TalonFX(moduleConstants.angleMotorId);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
       resetToAbsolute();

        // Drive Motor Config
        if (this.moduleNumber == 0) {
            mDriveMotor = new TalonFX(moduleConstants.driveMotorId);
            mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig1);
            mDriveMotor.getConfigurator().setPosition(0.0);
            mAngleMotor.setInverted(true);//TODO: try .getconfigurator
            mDriveMotor.setInverted(true);
        } else if (this.moduleNumber == 1) {
            
            mDriveMotor = new TalonFX(moduleConstants.driveMotorId);
            mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
            mDriveMotor.getConfigurator().setPosition(0.0);
            mAngleMotor.setInverted(true);
            mDriveMotor.setInverted(false);

        }   else if (this.moduleNumber == 2) {
            mDriveMotor = new TalonFX(moduleConstants.driveMotorId);
            mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
            mDriveMotor.getConfigurator().setPosition(0.0);
            mDriveMotor.setInverted(false);
            mAngleMotor.setInverted(true);
        }   else if (this.moduleNumber == 3) {
            mDriveMotor = new TalonFX(moduleConstants.driveMotorId);
            mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
            mDriveMotor.getConfigurator().setPosition(0.0);
            mDriveMotor.setInverted(false);
            mAngleMotor.setInverted(true);
        }

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                    Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANCoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
    }

    public double getDriveCurrent() {
        return mDriveMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getAngleCurrent() {
        return mAngleMotor.getSupplyCurrent().getValueAsDouble();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(),
                        Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
    }
}
