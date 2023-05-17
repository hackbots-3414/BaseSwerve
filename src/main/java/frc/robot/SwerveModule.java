package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    public double cancoderInitTime = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANCoder(moduleConstants.cancoderID, Constants.Swerve.canbusString);
        configAngleEncoder();

        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.Swerve.canbusString);
        configAngleMotor();

        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.Swerve.canbusString);
        configDriveMotor();

        lastAngle = getState().angle;

        waitForCanCoder();
    }

    private void waitForCanCoder(){
        /*
         * Wait for up to 1000 ms for a good CANcoder signal.
         *
         * This prevents a race condition during program startup
         * where we try to synchronize the Falcon encoder to the
         * CANcoder before we have received any position signal
         * from the CANcoder.
         */
        for (int i = 0; i < 100; ++i) {
            angleEncoder.getAbsolutePosition();
            if (angleEncoder.getLastError() == ErrorCode.OK) {
                break;
            }
            Timer.delay(0.010);            
            cancoderInitTime += 10;
        }
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    // private void setAngle(SwerveModuleState desiredState) {
    //     Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
    //             ? lastAngle
    //             : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    //     mAngleMotor.set(ControlMode.Position,
    //             Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
    //     lastAngle = angle;
    // }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d oldAngle = getAngle();
        angle = optimizeTurn(oldAngle, angle);  
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public double makePositiveDegrees(double anAngle) {
        double degrees = anAngle;
        degrees = degrees % 360;
        if (degrees < 0.0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double makePositiveDegrees(Rotation2d anAngle){
        return makePositiveDegrees(anAngle.getDegrees());
    }

    public Rotation2d optimizeTurn(Rotation2d oldAngle, Rotation2d newAngle){
        double steerAngle = makePositiveDegrees(newAngle);
        steerAngle %= (360);
        if (steerAngle < 0.0) {
            steerAngle += 360;
        }

        double difference = steerAngle - oldAngle.getDegrees();
        // Change the target angle so the difference is in the range [-360, 360) instead of [0, 360)
        if (difference >= 360) {
            steerAngle -= 360;
        } else if (difference < -360) {
            steerAngle += 360;
        }
        difference = steerAngle - oldAngle.getDegrees(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference >90 || difference < -90) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steerAngle += 180;
        }

        return Rotation2d.fromDegrees(makePositiveDegrees(steerAngle));
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(makePositiveDegrees(getCanCoder().getDegrees() - angleOffset.getDegrees()), Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault(Constants.IntakeConstants.canPause);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig, Constants.IntakeConstants.canPause);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault(Constants.IntakeConstants.canPause);
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig, Constants.IntakeConstants.canPause);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            System.out.println("Angle Neutral Mode Delay: " + e);
        }
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault(Constants.IntakeConstants.canPause);
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig, Constants.IntakeConstants.canPause);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            System.out.println("Drive Neutral Mode Delay: " + e);
        }
        mDriveMotor.setSelectedSensorPosition(0, 0, Constants.IntakeConstants.canPause);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference,
                        Constants.Swerve.driveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference,
                        Constants.Swerve.driveGearRatio),
                getAngle());
    }

    public double getSensorPosition() {
        return mDriveMotor.getSelectedSensorPosition();
    }

}